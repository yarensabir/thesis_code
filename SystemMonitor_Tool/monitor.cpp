#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/sysinfo.h>
#include <chrono>
#include <thread>
#include <csignal>

using namespace std;

// Global bayrak (Ctrl+C ile durdurmak için)
bool running = true;

void signalHandler(int signum) {
    cout << "\nKayıt durduruluyor..." << endl;
    running = false;
}

class SystemMonitor {
public:
    unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;

    SystemMonitor() {
        initCPU();
    }

    void initCPU() {
        FILE* file = fopen("/proc/stat", "r");
        fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow, &lastTotalSys, &lastTotalIdle);
        fclose(file);
    }

    double getCpuUsage() {
        double percent;
        FILE* file;
        unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

        file = fopen("/proc/stat", "r");
        fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow, &totalSys, &totalIdle);
        fclose(file);

        if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
            totalSys < lastTotalSys || totalIdle < lastTotalIdle) {
            percent = -1.0;
        } else {
            total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) +
                    (totalSys - lastTotalSys);
            percent = total;
            total += (totalIdle - lastTotalIdle);
            percent /= total;
            percent *= 100;
        }

        lastTotalUser = totalUser;
        lastTotalUserLow = totalUserLow;
        lastTotalSys = totalSys;
        lastTotalIdle = totalIdle;

        return percent;
    }

    double getRamUsage() {
        struct sysinfo memInfo;
        sysinfo(&memInfo);
        long long physMemUsed = memInfo.totalram - memInfo.freeram;
        physMemUsed *= memInfo.mem_unit;
        return (double)physMemUsed / (1024 * 1024);
    }

    double getTemperature() {
        ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
        if (!tempFile.is_open()) return 0.0;
        double temp;
        tempFile >> temp;
        tempFile.close();
        return temp / 1000.0;
    }
};

int main() {
    // Ctrl+C sinyalini yakala
    signal(SIGINT, signalHandler);

    SystemMonitor monitor;
    string filename = "performance_log_while_using_cam_with_ssh.csv";
    ofstream logFile(filename);

    // CSV Başlıkları
    logFile << "Time_Sec,CPU_Usage,RAM_Usage_MB,Temp_C" << endl;

    cout << "--- Sistem Monitörü Başlatıldı ---" << endl;
    cout << "Veriler '" << filename << "' dosyasına kaydediliyor." << endl;
    cout << "Durdurmak için CTRL+C yapın." << endl;

    auto startTime = chrono::steady_clock::now();

    while (running) {
        // Geçen süreyi hesapla
        auto now = chrono::steady_clock::now();
        double elapsed = chrono::duration_cast<chrono::duration<double>>(now - startTime).count();

        double cpu = monitor.getCpuUsage();
        double ram = monitor.getRamUsage();
        double temp = monitor.getTemperature();

        // CSV'ye yaz
        logFile << elapsed << "," << cpu << "," << ram << "," << temp << endl;

        // Ekrana bilgi bas (Opsiyonel)
        printf("Zaman: %.1fs | CPU: %%%.1f | RAM: %.0f MB | Sicaklik: %.1f C \r", elapsed, cpu, ram, temp);
        fflush(stdout);

        // 1 Saniye bekle (Veri sıklığını buradan ayarlayabilirsiniz)
        this_thread::sleep_for(chrono::seconds(1));
    }

    logFile.close();
    cout << "\nDosya kapatıldı. Güle güle!" << endl;
    return 0;
}