#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // 1. Kamerayı Başlat
    VideoCapture cap(0, CAP_V4L2); 

    if (!cap.isOpened()) {
        cerr << "HATA: Kamera açılamadı!" << endl;
        return -1;
    }

    // --- DÜZELTME 1: Giriş formatını (MJPG) ZORLAMIYORUZ ---
    // OpenCV'nin kameradan en temiz ham veriyi (genelde YUYV) alıp 
    // kendisinin BGR'a çevirmesine izin veriyoruz. Bu, renk kaymasını çözer.
    // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G')); // BU SATIRI SİLDİK

    // Çözünürlük İsteği
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // --- DÜZELTME 2: Kameranın GERÇEK boyutlarını öğreniyoruz ---
    // Biz 640 istesek bile kamera sürücüsü bazen 656x480 gibi paddingli verebilir.
    // Writer'ı tam bu boyuta göre açmazsak görüntü kayar (renkli çizgiler çıkar).
    double width = cap.get(CAP_PROP_FRAME_WIDTH);
    double height = cap.get(CAP_PROP_FRAME_HEIGHT);
    
    cout << "Kamera Gerçek Çözünürlüğü: " << width << "x" << height << endl;

    // --- DÜZELTME 3: Kayıt Formatını Değiştiriyoruz ---
    // .avi (MJPG) bazen bozulabilir. .mp4 (mp4v) daha güvenlidir.
    // Eğer Pi'de mp4v çalışmazsa tekrar MJPG deneyebiliriz.
    VideoWriter writer;
    string filename = "kayit_duzeltilmis.mp4";
    int codec = VideoWriter::fourcc('m', 'p', '4', 'v'); 
    double fps = 30.0; 

    writer.open(filename, codec, fps, Size((int)width, (int)height), true);

    if (!writer.isOpened()) {
        cerr << "HATA: Video dosyası açılamadı! Codec desteklenmiyor olabilir." << endl;
        // Alternatif olarak XVID deneyelim (Yedek Plan)
        codec = VideoWriter::fourcc('X', 'V', 'I', 'D');
        filename = "kayit_xvid.avi";
        writer.open(filename, codec, fps, Size((int)width, (int)height), true);
        if(!writer.isOpened()) return -1;
    }

    cout << "Temiz kayıt başladı: " << filename << endl;

    Mat frame;
    int frame_counter = 0;
    int max_frames = 300; // 10 saniye test

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        writer.write(frame);

        frame_counter++;
        if (frame_counter % 30 == 0) cout << "Kare: " << frame_counter << "\r" << flush;
        if (frame_counter >= max_frames) break;
    }

    cout << "\nKayıt tamamlandı." << endl;
    cap.release();
    writer.release();
    return 0;
}