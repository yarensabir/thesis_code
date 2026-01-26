
// /home/pi/data/source/
// /home/pi/data/result/
// /home/pi/data/result_csv/

#include <iostream>
#include "Stabilizer.h" 


using namespace std;

int main() {
    // =========================================
    // ---------------- AYARLAR ----------------

    string name_of_files = "test_3_kalman"; 

    // Girdi videosu (Offline test için)
    string videoPath = "/home/pi/data/source/TEST_1.mp4"; 
    
    // Çıktı Video İsimleri
    string outRT = "/home/pi/data/RT/RT_" + name_of_files + ".avi";
    //string outRT = "/home/pi/data/result/Result_RealTime.mp4";
    string outOffline = "/home/pi/data/OffLine/" + name_of_files + ".mp4" ;

    // Analiz Veri Dosyası (CSV) İsimleri 
    string csvRT = "/home/pi/data/RT/RT_" + name_of_files + ".csv";
    string csvOffline = "/home/pi/data/OffLine/" + name_of_files + ".csv";
    // ==========================================


    int mainChoice, subChoice, recordChoice;

    cout << "==========================================" << endl;
    cout << "VIDEO STABILIZATION SYSTEM" << endl;
    cout << "==========================================" << endl;
    cout << "1. Real-Time Stabilization (Pi Camera)" << endl;
    cout << "2. Offline Stabilization (Video File)" << endl;
    cout << "Secim: ";
    cin >> mainChoice;

    if (mainChoice == 1) {
        cout << "\n--- Real-Time Yontem Secimi ---" << endl;
        cout << "1. Sliding Window (Buffer Average)" << endl;
        cout << "2. Kalman Filter" << endl;
        cout << "Yontem: ";
        cin >> subChoice;

        cout << "\n--- Kayit Modu Secimi ---" << endl;
        cout << "0. Standart Kayit (Sadece Stabilize Video)" << endl;
        cout << "1. Ayri Dosyalar (Raw.avi + Result.avi)" << endl;
        cout << "2. Yan Yana (Side-by-Side Tek Video, Sol Kare Ham Görüntü - Sağ Kare Stabilize Görüntü)" << endl;
        cout << "Mod: ";
        cin >> recordChoice;

        RealTimeMethod selectedMethod = static_cast<RealTimeMethod>(subChoice);
        RecordMode selectedMode = static_cast<RecordMode>(recordChoice);
        
        if(subChoice == 1) runRealTimeStabilization(RT_SLIDING_WINDOW, outRT, csvRT, selectedMode);
        else if(subChoice == 2) runRealTimeStabilization(RT_KALMAN_FILTER, outRT, csvRT, recordChoice);
        else cout << "Hatali secim!" << endl;

    } else if (mainChoice == 2) {
        cout << "\n--- Offline Yontem Secimi ---" << endl;
        cout << "1. Moving Average (Basit)" << endl;
        cout << "2. Gaussian Smoothing (Sinematik)" << endl;
        cout << "Yontem: ";
        cin >> subChoice;

        if(subChoice == 1) runOfflineStabilization(videoPath, OFF_MOVING_AVERAGE, outOffline, csvOffline);
        else if(subChoice == 2) runOfflineStabilization(videoPath, OFF_GAUSSIAN, outOffline, csvOffline);
        else cout << "Hatali secim!" << endl;

    } else {
        cout << "Gecersiz ana secim!" << endl;
    }

    return 0;
}