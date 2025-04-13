# ESP32-CAM ve Ultrasonik Sensör ile Akıllı Engel Algılayan Robot

Bu proje, çizgi takibi yapan ve önüne engel çıktığında durabilen veya yön değiştirebilen bir mobil robot geliştirmek amacıyla tasarlanmıştır. ESP32-CAM üzerinden görüntü aktarımı sağlanırken, Arduino üzerinden HC-SR04 ultrasonik sensör yardımıyla engel algılama gerçekleştirilir.

## Proje Amaçları

- Gerçek zamanlı görüntü aktarımı (ESP32-CAM ile)
- Ultrasonik sensör ile mesafe ölçümü ve engel tespiti
- Otonom hareket kabiliyeti (durma veya yön değiştirme)
- Gömülü sistem programlama pratiği kazanmak

## Dosyalar

- `ESP32CAM.ino`  
  ESP32-CAM üzerinden canlı görüntü aktarımı gerçekleştiren kod. Bu kod ile robotun çevresi hakkında gerçek zamanlı görsel bilgi alınabilir. Web sunucu üzerinden görüntü izlenebilir.

- `arduino.ino`  
  Arduino Uno üzerinde çalışan ve HC-SR04 sensör yardımıyla mesafe ölçen kod. Mesafe belirli bir eşik değerin altına düştüğünde motorlar durdurulur veya yön değiştirme sinyali verilir.

## Kullanılan Donanımlar

- ESP32-CAM
- Arduino Uno
- HY-SRF05 Ultrasonik Mesafe Sensörü
- L298N Motor Sürücü Kartı
- DC Motorlar
- Jumper Kablolar
- 9V Güç Kaynağı


## Kurulum Talimatları

1. Arduino IDE'yi indirip kurun.
2. Gerekli kart sürücülerini (ESP32 için) kurun.
3. `ESP32CAM.ino` dosyasını ESP32-CAM’e, `arduino.ino` dosyasını Arduino Uno’ya yükleyin.
4. ESP32-CAM'in bağlı olduğu Wi-Fi ağ bilgilerini kodda güncelleyin.
5. Devre bağlantılarını doğru bir şekilde yapın.
6. Cihazları çalıştırın ve işlevleri test edin.

## Özellikler

- Engel algılama menzili: 2cm – 400cm
- Görüntü çözünürlüğü: 320x240 (isteğe göre artırılabilir)
- Kolay devre kurulumu
- Hem görüntü hem mesafe tabanlı kontrol

---

# Smart Obstacle-Avoiding Robot with ESP32-CAM and Ultrasonic Sensor

This project aims to build a mobile robot that follows a line and can stop or change direction when it encounters an obstacle. The ESP32-CAM module handles video streaming, while the Arduino Uno controls the ultrasonic sensor for obstacle detection.

## Project Goals

- Real-time video streaming using ESP32-CAM
- Obstacle detection via ultrasonic distance measurement
- Autonomous behavior (stop or change direction)
- Practice in embedded systems programming

## Files

- `ESP32CAM.ino`  
  Contains code for the ESP32-CAM to transmit live video. This enables the robot to provide visual feedback through a web interface.

- `arduino.ino`  
  Runs on Arduino Uno and uses the HC-SR04 ultrasonic sensor to measure distance. When an obstacle is detected within a certain range, motors are stopped or redirected.

## Hardware Used

- ESP32-CAM
- Arduino Uno
- HY-SRF05 Ultrasonic Sensor
- L298N Motor Driver
- DC Motors
- Jumper Wires
- 9V Power Supply


## Setup Instructions

1. Download and install the Arduino IDE.
2. Add the necessary board support for ESP32.
3. Upload `ESP32CAM.ino` to the ESP32-CAM, and `arduino.ino` to the Arduino Uno.
4. Update your Wi-Fi SSID and password in the ESP32 code.
5. Connect all hardware components as per the schematic.
6. Power up the devices and test the system.

## Features

- Obstacle detection range: 2cm – 400cm
- Video resolution: 320x240 (can be changed)
- Simple and compact hardware design
- Dual sensing: image and distance-based control
