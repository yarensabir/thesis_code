#include "Stabilizer.h"
#include "MotionEstimator.h"
#include "Transform.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <deque>
#include <fstream>
#include <vector>
#include <cmath>
#include <csignal>
#include <string>

using namespace cv;
using namespace std;

// Global durdurma bayrağı
volatile bool stop_flag = false;

// Ctrl+C (SIGINT) sinyalini yakalayan fonksiyon
void signalHandler(int signum) {
    cout << "\n\n>>> DURDURMA SINYALI ALINDI (CTRL+C) <<<" << endl;
    cout << "Video dosyasi kapatiliyor, lutfen bekleyin..." << endl;
    stop_flag = true; // Döngüyü kıracak bayrağı aktif et
}

struct MotionKalman {
    KalmanFilter KF;
    Mat state, meas;
    MotionKalman() {
        KF.init(6, 3, 0); 
        setIdentity(KF.transitionMatrix);
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e-5)); 
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); 
        setIdentity(KF.errorCovPost, Scalar::all(1));
        meas = Mat::zeros(3, 1, CV_32F);
    }
    TransformParam update(TransformParam raw) {
        KF.predict();
        meas.at<float>(0) = (float)raw.dx; meas.at<float>(1) = (float)raw.dy; meas.at<float>(2) = (float)raw.da;
        Mat estimated = KF.correct(meas);
        return TransformParam(estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2));
    }
};

// ---------------------------------------------------------
// 1. GERÇEK ZAMANLI FONKSİYON
// ---------------------------------------------------------
void runRealTimeStabilization(int method, string outputPath, string csvPath, int recordMode) {
    // -----------------------------------------------------------------------
    // 1. KAMERA AYARLARI (Sizin GStreamer Pipeline'ınız)
    // -----------------------------------------------------------------------
    // 1. Sinyal Yakalayıcıyı Aktif Et
    signal(SIGINT, signalHandler);
    stop_flag = false; // Her çalıştırışta sıfırla
    // 2. Kamera Ayarları
    string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink";
    VideoCapture cap(pipeline, CAP_GSTREAMER);

    if(!cap.isOpened()) {
        cerr << "Hata: Kamera acilamadi!" << endl;
        return;
    }

    // GStreamer kullandığımız için boyutları manuel sabitliyoruz (Pipeline ile uyumlu)
    int w = 640; 
    int h = 480;
    double fps = 30.0;

    // -----------------------------------------------------------------------
    // 2. VIDEO WRITER AYARLARI (Seçilen Mod'a Göre)
    // -----------------------------------------------------------------------
    VideoWriter writerStandard;   // Mod 0 ve 1 için (Stabilize Görüntü)
    VideoWriter writerRaw;        // Mod 1 için (Ham Görüntü)
    VideoWriter writerSideBySide; // Mod 2 için (Yan Yana Görüntü)
    
    // Codec Seçimi: Raspberry Pi'de genelde MJPG veya mp4v verimlidir
    //int codec = VideoWriter::fourcc('X', 'V', 'I', 'D');
    //int codec = 0;
    int codec = VideoWriter::fourcc('M', 'P', '4', 'V');

    if (recordMode == 0) { 
        if(!writerStandard.isOpened()) {
            cerr << "HATA: writerStandard acilamadi! Codec: " << codec << " Yol: " << outputPath << endl;
            return;
        }
        // Mod 0: Standart (Sadece Sonuç)
        writerStandard.open(outputPath, codec, fps, Size(w, h), true);
        cout << "Kayit Modu: Standart (Sadece Stabilize)" << endl;
    } 
    else if (recordMode == 1) { 
        // Mod 1: Ayrı Dosyalar (Ham + Sonuç)
        writerStandard.open(outputPath, codec, fps, Size(w, h), true);
        
        // Raw dosya ismini otomatik üret (örn: result.mp4 -> result_RAW.avi)
        string rawOut = outputPath.substr(0, outputPath.find_last_of('.')) + "_RAW.avi";
        writerRaw.open(rawOut, codec, fps, Size(w, h), true);
        cout << "Kayit Modu: Ayri Dosyalar (" << rawOut << " + Stabilize)" << endl;
    } 
    else if (recordMode == 2) { 
        // Mod 2: Yan Yana (Side-by-Side)
        // Genişlik 2 katı (w * 2) çünkü iki görüntü yan yana gelecek
        writerSideBySide.open(outputPath, codec, fps, Size(w * 2, h), true);
        cout << "Kayit Modu: Yan Yana (Side-by-Side)" << endl;
    }

    // -----------------------------------------------------------------------
    // 3. DEĞİŞKENLER VE HAZIRLIK
    // -----------------------------------------------------------------------
    // CSV Dosyası
    ofstream csvFile(csvPath);
    csvFile << "frame,dx,dy,da,raw_x,raw_y,smooth_x,smooth_y" << endl;

    Mat curr, currGray, prev, prevGray;
    
    // İlk kareyi oku
    cap >> prev;
    if(prev.empty()) {
        cerr << "Hata: Ilk kare okunamadi!" << endl;
        return;
    }
    cvtColor(prev, prevGray, COLOR_BGR2GRAY);

    // Yörünge Değişkenleri
    double x = 0, y = 0, a = 0;
    double smooth_x = 0, smooth_y = 0, smooth_a = 0;

    // Kalman Filtresi Nesnesi (Struct dosyanın başında tanımlı olduğu için burada çağırıyoruz)
    MotionKalman kf; 

    int frame_counter = 0;

    cout << "Stabilizasyon basladi. Durdurmak icin CTRL+C..." << endl;

    // -----------------------------------------------------------------------
    // 4. ANA DÖNGÜ
    // -----------------------------------------------------------------------
// 4. ANA DÖNGÜ
    while(!stop_flag) {
        cap >> curr;
        if(curr.empty()) break;
        
        cvtColor(curr, currGray, COLOR_BGR2GRAY);

        // --- Optik Akış (Optical Flow) ---
        vector<Point2f> prevPts, currPts;
        goodFeaturesToTrack(prevGray, prevPts, 200, 0.01, 30);
        
        bool motionDetected = false;
        double dx = 0, dy = 0, da = 0;

        if(prevPts.size() > 0) {
            vector<uchar> status;
            vector<float> err;
            calcOpticalFlowPyrLK(prevGray, currGray, prevPts, currPts, status, err);
            
            vector<Point2f> pA, pB;
            for(size_t i=0; i < status.size(); i++) {
                if(status[i]) {
                    pA.push_back(prevPts[i]);
                    pB.push_back(currPts[i]);
                }
            }

            if(pA.size() > 10) {
                Mat T = estimateAffinePartial2D(pA, pB);
                if(!T.empty()) {
                    dx = T.at<double>(0, 2);
                    dy = T.at<double>(1, 2);
                    da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
                    motionDetected = true;
                }
            }
        }

        if (motionDetected) {
            x += dx; 
            y += dy; 
            a += da;

            if (method == RT_KALMAN_FILTER) {
                TransformParam rawP(dx, dy, da);
                TransformParam smoothP = kf.update(rawP);
                smooth_x += smoothP.dx;
                smooth_y += smoothP.dy;
                smooth_a += smoothP.da;
            } 
            else { 
                smooth_x = smooth_x * 0.9 + x * 0.1;
                smooth_y = smooth_y * 0.9 + y * 0.1;
                smooth_a = smooth_a * 0.9 + a * 0.1;
            }

            double diff_x = smooth_x - x;
            double diff_y = smooth_y - y;
            double diff_a = smooth_a - a;

            double target_dx = dx + diff_x;
            double target_dy = dy + diff_y;
            double target_da = da + diff_a;

            Mat T_new = Mat::eye(2, 3, CV_64F);
            T_new.at<double>(0, 0) = cos(target_da); 
            T_new.at<double>(0, 1) = -sin(target_da);
            T_new.at<double>(1, 0) = sin(target_da); 
            T_new.at<double>(1, 1) = cos(target_da);
            T_new.at<double>(0, 2) = target_dx; 
            T_new.at<double>(1, 2) = target_dy;

            Mat stabilizedFrame;
            warpAffine(curr, stabilizedFrame, T_new, curr.size());

            csvFile << frame_counter << "," << dx << "," << dy << "," << da << "," 
                    << x << "," << y << "," << smooth_x << "," << smooth_y << endl;

            // --- KAYIT MANTIĞI DÜZELTİLDİ ---
            if (recordMode == 0) {
                if (writerStandard.isOpened()) writerStandard.write(stabilizedFrame);
                else cerr << "Hata: writerStandard acilamadi!" << endl;
            }
            else if (recordMode == 1) {
                if (writerRaw.isOpened()) writerRaw.write(curr);
                else cerr << "Hata: writerRaw acilamadi!" << endl;

                if (writerStandard.isOpened()) writerStandard.write(stabilizedFrame);
                else cerr << "Hata: writerStandard acilamadi!" << endl;
            }
            else if (recordMode == 2) {
                if (writerSideBySide.isOpened()) {
                    Mat combined;
                    hconcat(curr, stabilizedFrame, combined);
                    writerSideBySide.write(combined);
                }
                else cerr << "Hata: writerSideBySide acilamadi!" << endl;
            }
        } 
        else {
            // Hareket yoksa orjinal kareleri yaz
            if (recordMode == 0 && writerStandard.isOpened()) writerStandard.write(curr);
            else if (recordMode == 1) {
                if(writerRaw.isOpened()) writerRaw.write(curr);
                if(writerStandard.isOpened()) writerStandard.write(curr);
            }
            else if (recordMode == 2 && writerSideBySide.isOpened()) {
                Mat combined;
                hconcat(curr, curr, combined);
                writerSideBySide.write(combined);
            }
        }

        curr.copyTo(prev);
        currGray.copyTo(prevGray);
        frame_counter++;
        
        if(frame_counter % 30 == 0) cout << "Kare: " << frame_counter << "\r" << flush;
    } // while döngüsünün sonu

// --- TEMİZLİK ---
    // Önce yazıcıları kapat ki buffer'daki veriler diske yazılsın
    if(writerStandard.isOpened()) {
        cout << "Standard video kaydediliyor..." << endl;
        writerStandard.release();
    }
    if(writerRaw.isOpened()) writerRaw.release();
    if(writerSideBySide.isOpened()) writerSideBySide.release();
    
    csvFile.close();
    
    // En son kamerayı kapat (Takılmaya en meyilli kısım budur)
    cout << "Kamera kapatiliyor..." << endl;
    cap.release(); 
    
    cout << "\nStabilizasyon bitti. Dosyalar kaydedildi." << endl;
}

void runRealTimeStabilization_old(RealTimeMethod method, string outputName, string logName) {
    cout << "[Real-Time] Pi Kamera GStreamer ile aciliyor..." << endl;
    
    // ... (Pipeline ve VideoCapture aynı) ...
    //string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink";
    string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink drop=true max-buffers=1";
    
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if(!cap.isOpened()) { cerr << "Hata: Kamera acilamadi!" << endl; return; }
    
    Mat curr; cap >> curr;
    if (curr.empty()) return;
    int w = curr.cols; int h = curr.rows;

    VideoWriter writer(outputName, VideoWriter::fourcc('m','p','4','v'), 30, Size(w, h));
    
    //  Dosya ismi parametreden alınıyor (c_str() gerekli çünkü fopen C fonksiyonu)
    FILE* fp = fopen(logName.c_str(), "w");
    // Başlığa FPS ve Time eklendi
    fprintf(fp, "frame,raw_x,smooth_x,fps,process_time_ms\n");

    MotionEstimator estimator;
    MotionKalman kf; 
    
    int BUFFER_SIZE = 30; 
    deque<TransformParam> motion_buffer;
    deque<Mat> frame_buffer;
    TransformParam cumulative_motion(0,0,0);
    TransformParam smoothed_pos(0,0,0); 

    estimator.initialize(curr);
    motion_buffer.push_back(TransformParam(0,0,0));
    frame_buffer.push_back(curr.clone());

    int frame_idx = 0;
    cout << "Islem basliyor... Video: " << outputName << " | Log: " << logName << endl;

    while(true) {
        double t_start = (double)cv::getTickCount(); // Zaman Başla

        cap >> curr;
        if(curr.empty()) break;

        TransformParam motion = estimator.estimate(curr);
        cumulative_motion += motion;

        if (method == RT_KALMAN_FILTER) smoothed_pos = kf.update(cumulative_motion);
        
        motion_buffer.push_back(cumulative_motion);
        frame_buffer.push_back(curr.clone());

        if(frame_buffer.size() > BUFFER_SIZE) {
            if (method == RT_SLIDING_WINDOW) {
                TransformParam sum(0,0,0);
                for(const auto& m : motion_buffer) sum += m;
                smoothed_pos = sum / (double)motion_buffer.size();
            }

            Mat target_frame = frame_buffer.front();
            TransformParam target_pos = motion_buffer.front(); 
            TransformParam diff = target_pos - smoothed_pos;
            
            Mat M = getAffineFromParam(diff);
            Mat stabilized;
            warpAffine(target_frame, stabilized, M, target_frame.size());

            writer.write(stabilized);

            // Zaman Bitiş ve FPS Hesaplama
            double t_end = (double)cv::getTickCount();
            double time_ms = ((t_end - t_start) / cv::getTickFrequency()) * 1000.0;
            double fps = 1000.0 / time_ms;

            // GÜNCELLEME: target_pos kullanılarak doğru loglama
            fprintf(fp, "%d, %f, %f, %.2f, %.2f\n", frame_idx, target_pos.dx, smoothed_pos.dx, fps, time_ms);

            frame_buffer.pop_front();
            motion_buffer.pop_front();
            frame_idx++;
        }
    }
    // Kalan buffer boşaltma
    while(!frame_buffer.empty()) {
        writer.write(frame_buffer.front());
        frame_buffer.pop_front();
    }
    fclose(fp);
    cout << "RealTime Bitti." << endl;
}

// ---------------------------------------------------------
// 2. ÇEVRİMDIŞI FONKSİYON
// ---------------------------------------------------------
void runOfflineStabilization(string videoPath, OfflineMethod method, string outputName, string logName) {
    cout << "[Offline] Analiz yapiliyor..." << endl;
    VideoCapture cap(videoPath);
    if(!cap.isOpened()) { cerr << "Dosya yok!" << endl; return; }

    MotionEstimator estimator;
    Mat curr;
    vector<TransformParam> trajectory; 
    vector<TransformParam> transforms; 
    TransformParam cumulative(0,0,0);

    cap >> curr;
    estimator.initialize(curr);

    // PASS 1: Veri Topla
    while(true) {
        cap >> curr;
        if(curr.empty()) break;
        TransformParam motion = estimator.estimate(curr);
        transforms.push_back(motion);
        cumulative += motion;
        trajectory.push_back(cumulative);
    }
    
    // PASS 2: Yumuşatma
    vector<TransformParam> smoothed_trajectory;
    int RADIUS = 30; 

    for(size_t i=0; i<trajectory.size(); i++) {
        TransformParam sum(0,0,0);
        double total_weight = 0;

        for(int j=-RADIUS; j<=RADIUS; j++) {
            if(i+j >= 0 && i+j < trajectory.size()) {
                if (method == OFF_MOVING_AVERAGE) {
                    sum += trajectory[i+j];
                    total_weight += 1.0;
                } 
                else if (method == OFF_GAUSSIAN) {
                    double sigma = RADIUS / 3.0;
                    double weight = exp(-(j*j) / (2 * sigma * sigma));
                    TransformParam p = trajectory[i+j];
                    sum.dx += p.dx * weight; sum.dy += p.dy * weight; sum.da += p.da * weight;
                    total_weight += weight;
                }
            }
        }
        if (method == OFF_MOVING_AVERAGE) smoothed_trajectory.push_back(sum / total_weight);
        else smoothed_trajectory.push_back(TransformParam(sum.dx/total_weight, sum.dy/total_weight, sum.da/total_weight));
    }

    // GÜNCELLEME: CSV Kaydı (Parametreden gelen 'logName' kullanılıyor)
    cout << "Veriler kaydediliyor: " << logName << endl;
    ofstream logFile(logName);
    logFile << "frame,raw_x,raw_y,raw_a,smooth_x,smooth_y,smooth_a" << endl;
    for(size_t i=0; i < trajectory.size(); i++) {
        logFile << i << "," 
                << trajectory[i].dx << "," << trajectory[i].dy << "," << trajectory[i].da << ","
                << smoothed_trajectory[i].dx << "," << smoothed_trajectory[i].dy << "," << smoothed_trajectory[i].da 
                << endl;
    }
    logFile.close();

    // PASS 3: Video Yazma
    cout << "Video olusturuluyor: " << outputName << endl;
    cap.release(); cap.open(videoPath);
    int w = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    
    VideoWriter writer(outputName, VideoWriter::fourcc('m','p','4','v'), 30, Size(w,h));
    cap >> curr; 
    for(size_t i=0; i<transforms.size(); i++) {
        cap >> curr;
        if(curr.empty()) break;
        TransformParam diff = smoothed_trajectory[i] - trajectory[i];
        Mat M = getAffineFromParam(diff);
        Mat stabilized;
        warpAffine(curr, stabilized, M, curr.size());
        writer.write(stabilized);
    }
    cout << "Offline Bitti." << endl;
}