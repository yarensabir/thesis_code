#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>

using namespace cv;
using namespace std;

int main() {
    cout << "1. GStreamer testi basliyor..." << endl;
    
    // En basit pipeline
    string pipeline = "libcamerasrc ! videoconvert ! appsink";
    
    cout << "2. Pipeline: " << pipeline << endl;
    
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    
    if(!cap.isOpened()) {
        cerr << "HATA: Kamera acilamadi!" << endl;
        cerr << "Asagidaki komutlari deneyin:" << endl;
        cerr << "  sudo systemctl status systemd-udevd" << endl;
        cerr << "  groups pi" << endl;
        cerr << "  ls -la /dev/video*" << endl;
        return -1;
    }
    
    cout << "3. Kamera acildi. 3 kare okumaya calisiyor..." << endl;
    
    for(int i = 0; i < 3; i++) {
        Mat frame;
        cap >> frame;
        
        if(frame.empty()) {
            cerr << "4. HATA: Kare " << i << " bos geldi!" << endl;
        } else {
            cout << "4. Kare " << i << " alindi. Boyut: " 
                 << frame.cols << "x" << frame.rows << endl;
        }
        
        // 1 saniye bekle
        waitKey(1000);
    }
    
    cout << "5. Test tamamlandi. Kamera kapatiliyor..." << endl;
    cap.release();
    
    cout << "6. Alternatif test: v4l2" << endl;
    system("v4l2-ctl --list-devices");
    
    return 0;
}