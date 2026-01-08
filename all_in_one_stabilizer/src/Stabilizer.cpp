#include "Stabilizer.h"
#include "MotionEstimator.h"
#include "Transform.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <deque>
#include <fstream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

// ... (MotionKalman struct'ı aynı kalacak, buraya tekrar yazmadım yer kaplamasın diye) ...
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
// GÜNCELLEME: 'logName' parametresi eklendi
void runRealTimeStabilization(RealTimeMethod method, string outputName, string logName) {
    cout << "[Real-Time] Pi Kamera GStreamer ile aciliyor..." << endl;
    
    // ... (Pipeline ve VideoCapture aynı) ...
    string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if(!cap.isOpened()) { cerr << "Hata: Kamera acilamadi!" << endl; return; }

    Mat curr; cap >> curr;
    if (curr.empty()) return;
    int w = curr.cols; int h = curr.rows;

    VideoWriter writer(outputName, VideoWriter::fourcc('m','p','4','v'), 30, Size(w, h));
    
    // GÜNCELLEME: Dosya ismi parametreden alınıyor (c_str() gerekli çünkü fopen C fonksiyonu)
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
// GÜNCELLEME: 'logName' parametresi eklendi
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