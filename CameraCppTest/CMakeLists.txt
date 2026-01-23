cmake_minimum_required(VERSION 3.0)
project(CameraTest)

# OpenCV paketini bul
find_package(OpenCV REQUIRED)

# Derlenecek dosyayı ve çıktı ismini belirle
add_executable(CameraTest main.cpp)

# Kütüphaneleri bağla (Link)
target_link_libraries(CameraTest ${OpenCV_LIBS})

# Yeni test dosyası için hedef oluştur
#add_executable(camera_test test_camera.cpp)

