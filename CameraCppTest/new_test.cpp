#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono> // Zaman kontrolü için

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) return -1;

    // Kameranın ısınması için kısa bir bekleme (ÖNEMLİ)
    cout << "Kamera hazirlaniyor..." << endl;
    for(int i=0; i<10; i++) { Mat dummy; cap >> dummy; } 

    double width = cap.get(CAP_PROP_FRAME_WIDTH);
    double height = cap.get(CAP_PROP_FRAME_HEIGHT);

    VideoWriter writer("test_5sn.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), 20.0, Size((int)width, (int)height), true);

    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;

    cout << "Kayit basladi (5 saniye sürecek)..." << endl;

    while (true) {
        Mat frame;
        cap >> frame; // Kareyi oku

        if (frame.empty()) continue;

        writer.write(frame);
        frame_count++;

        // 5 saniye doldu mu kontrol et
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();
        
        if (elapsed >= 5.0) break; 
    }

    cout << "Bitti! Toplam kaydedilen kare: " << frame_count << endl;
    cout << "Gerçek FPS: " << frame_count / 5.0 << endl;

    cap.release();
    writer.release();
    return 0;
}