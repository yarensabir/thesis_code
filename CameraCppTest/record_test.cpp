#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) return -1;

    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // Kameranın ısınması ve ışık ayarı için 2 saniye bekle
    cout << "Kamera isigi ayarlandiginda kayit baslayacak..." << endl;
    Mat dummy;
    for(int i=0; i<30; i++) cap >> dummy;

    int width = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int height = (int)cap.get(CAP_PROP_FRAME_HEIGHT);

    // --- HIZ SORUNU ÇÖZÜMÜ ---
    // Eğer video çok hızlıysa, buradaki 20.0 değerini düşürmeliyiz.
    // Pi 4 üzerinde v1.3 kamera genelde 10-15 FPS civarı stabil çalışır.
    double save_fps = 30.0; 
    VideoWriter writer("gercek_zamanli.avi", VideoWriter::fourcc('X', 'V', 'I', 'D'), save_fps, Size(width, height), true);

    Mat frame;
    cout << "Kayit basladi (100 kare aliniyor)..." << endl;

    for(int i=0; i<100; i++) {
        auto start = chrono::steady_clock::now();
        
        cap >> frame;
        if(frame.empty()) break;
        
        writer.write(frame);

        // Her 10 karede bir ilerlemeyi göster
        if(i % 10 == 0) cout << "Kare: " << i << endl;

        // İşlemci çok hızlıysa kameranın yetişmesi için küçük bir bekleme
        auto end = chrono::steady_clock::now();
        chrono::duration<double, milli> elapsed = end - start;
        if(elapsed.count() < (1000.0 / save_fps)) {
            int delay = (1000.0 / save_fps) - elapsed.count();
            waitKey(delay); 
        }
    }

    writer.release();
    cap.release();
    cout << "Kayit tamamlandi. Simdi videoyu kontrol et." << endl;
    return 0;
}