#pragma once
#include <string>

enum RealTimeMethod {
    RT_SLIDING_WINDOW = 1,
    RT_KALMAN_FILTER = 2
};

enum OfflineMethod {
    OFF_MOVING_AVERAGE = 1,
    OFF_GAUSSIAN = 2
};

enum RecordMode {
    REC_ONLY_STABILIZED = 0,    // Sadece stabilize videoyu kaydet
    REC_SEPARATE_FILES = 1,     // Raw ve Stabilize'yi ayrı dosyalara kaydet
    REC_SIDE_BY_SIDE = 2        // İkisini yan yana tek videoya kaydet
};
void signalHandler(int signum);
void runRealTimeStabilization(int method,  std::string outputPath,  std::string csvPath, int recordMode);
void runRealTimeStabilization_old(RealTimeMethod method,  std::string outputName,  std::string logName);
void runOfflineStabilization(std::string inputPath, OfflineMethod method,  std::string outputName,  std::string logName);