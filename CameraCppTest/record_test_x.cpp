#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // 1. Kamerayı V4L2 üzerinden aç (libcamerify ile çalışacak)
    VideoCapture cap(0, CAP_V4L2); 

    if (!cap.isOpened()) {
        cerr << "HATA: Kamera acilamadi! Lutfen baglantilari kontrol edin." << endl;
        return -1;
    }

    // Çözünürlük isteği (v1.3 için ideal 640x480)
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // --- KRİTİK ADIM: Donanımın GERÇEKTE ne verdiğini alıyoruz ---
    // Eğer sürücü 640 yerine 656 verirse, Writer'ı buna göre açarak görüntü kaymasını önleriz.
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    cout << "Kamera Cozunurlugu: " << frame_width << "x" << frame_height << endl;

    // 2. Video Yazıcıyı Ayarla (XVID + .avi en az takılan formattır)
    VideoWriter writer("sunum_kaydi.avi", 
                       VideoWriter::fourcc('X', 'V', 'I', 'D'), 
                       20.0, 
                       Size(frame_width, frame_height), 
                       true);

    if (!writer.isOpened()) {
        cerr << "HATA: Video dosyasi yazmak icin acilamadi!" << endl;
        return -1;
    }

    Mat frame;
    cout << "\n>>> KAYIT BASLADI <<<" << endl;
    cout << "Durdurmak ve kaydetmek icin pencere acikken 'q' tusuna basin." << endl;

    while (true) {
        cap >> frame; // Kameradan kareyi oku
        if (frame.empty()) {
            cerr << "Bos kare atlandi..." << endl;
            continue;
        }

        // Kareyi dosyaya yaz
        writer.write(frame);

        // Ekranda göster (Görüntünün geldiğini teyit etmek için)
        imshow("Kamera Testi - Cikis icin 'q'", frame);

        // 'q' tuşuna basılıp basılmadığını kontrol et
        char c = (char)waitKey(1);
        if (c == 'q' || c == 'Q') {
            cout << "\n'q' basildi. Kayit sonlandiriliyor..." << endl;
            break;
        }
    }

    // --- GÜVENLİ KAPATMA PROTOKOLÜ ---
    // Dünkü bozuk dosya sorunu burada düzgün kapatılmadığı için oluyordu.
    writer.release(); 
    cap.release();
    destroyAllWindows();

    cout << "Dosya basariyla kaydedildi: sunum_kaydi.avi" << endl;
    return 0;
}