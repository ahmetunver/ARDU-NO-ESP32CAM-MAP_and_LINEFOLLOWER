/*
 * ESP32-CAM Çizgi Takibi ve Haritalama Sistemi
 * İyileştirilmiş siyah çizgi takibi algoritması ve gelişmiş haritalama
 */

#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"
#include "SPI.h"
#include "SD.h"

// Kamera pin tanımlamaları
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Arduino'ya gönderilecek komutlar
#define CMD_ILERI  'F'  // Forward
#define CMD_GERI   'B'  // Backward
#define CMD_SOL    'L'  // Left
#define CMD_SAG    'R'  // Right
#define CMD_DUR    'S'  // Stop

// SD kart yapılandırması
#define SD_CS 13

// Harita konfigürasyonu - boyutu artırıldı
#define HARITA_BOYUTU 200        // 200x200 hücreli harita
#define HUCRE_BOYUTU 2           // Her hücre 2x2 cm (daha yüksek çözünürlük)
#define HARITA_MERKEZI 100        // Haritanın merkezi (100,100)

// Konum ve yön için değişkenler
float x_pos = 0;
float y_pos = 0;
float yon = 0;        // Yön (derece, 0 = kuzey, saat yönünde artar)

// Hız tahminleme için değişkenler
float v_x = 0;        // X yönündeki tahmini hız
float v_y = 0;        // Y yönündeki tahmini hız
float hizlanma = 0.1; // Hızlanma faktörü

// Harita verisi için dinamik dizi
uint8_t **harita = NULL;

// Zamanlama değişkenleri
unsigned long sonKayitZamani = 0;
const unsigned long kayitAraligi = 3000; // 3 saniyede bir kaydet (daha sık)

unsigned long sonKomutZamani = 0;
const unsigned long komutAraligi = 80; // 80ms'de bir komut gönder (daha hızlı tepki)

unsigned long sonFotografZamani = 0;
const unsigned long fotografAraligi = 30000; // 30 saniyede bir fotoğraf

// Çizgi takibi değişkenleri
int cizgiX = 0;         // Çizginin yatay konumu
int cizgiGenislik = 0;  // Çizginin genişliği
bool cizgiBulundu = false;
unsigned long sonCizgiZamani = 0;

// Yumuşak dönüş için yardımcı değişkenler
int oncekiSapma = 0;
float kumulatifSapma = 0;
float P_katsayi = 1.0;   // Orantısal kontrol katsayısı
float I_katsayi = 0.1;   // İntegral kontrol katsayısı 
float D_katsayi = 0.3;   // Türevsel kontrol katsayısı

// Son hareket komutunu sakla
char sonKomut = CMD_DUR;
char stabilKomut = CMD_DUR;  // Daha düzgün komut geçişleri için
int komutSayaci = 0;         // Aynı komutun tekrarı için sayaç

// Kalman filtresi benzeri konum takibi için değişkenler
float konum_guven = 1.0;     // Konum güven faktörü (0-1 arası)
unsigned long sonKonumGuncellemeZamani = 0;

// Fonksiyon prototipleri
void haritayiIlklendir();
void haritayiKaydet();
void fotografCek();
bool cizgiAlgila();
char cizgiKomutunuBelirle();
void konumuGuncelle(char komut);
void haritayiGuncelle();
void temizle();

void setup() {
  // Seri portu başlat
  Serial.begin(9600);
  Serial.println("ESP32-CAM Çizgi Takibi ve Haritalama Sistemi başlatılıyor...");
  
  // Kamera yapılandırması
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Grayscale formatı çizgi tespiti için daha iyi
  
  // QVGA çözünürlükte başla (çizgi takibi için yeterli ve daha hızlı)
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;  // Daha düşük kalite, daha hızlı işlem
  config.fb_count = 1;       // Tek frame buffer kullan (grayscale için daha hızlı)
  
  // Kamerayı başlat
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera başlatılamadı, hata: 0x%x\n", err);
    return;
  }
  
  // Kamera ayarlarını optimize et
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    // Siyah çizgi tespiti için optimize edilmiş ayarlar
    s->set_contrast(s, 2);        // Maksimum kontrast
    s->set_brightness(s, 0);      // Orta parlaklık
    s->set_saturation(s, -2);     // Minimum doygunluk (siyah-beyaz benzeri)
    s->set_gainceiling(s, GAINCEILING_2X);
    s->set_exposure_ctrl(s, 1);   // Otomatik pozlama açık
    s->set_aec2(s, 1);            // Otomatik pozlama düzeltme açık
    s->set_ae_level(s, 0);        // Otomatik pozlama seviyesi
    
    Serial.println("Kamera parametreleri siyah çizgi tespiti için optimize edildi");
  }
  
  // SD kartı başlat
  if (!SD_MMC.begin()) {
    Serial.println("SD kart başlatılamadı!");
  } else {
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("SD kart takılı değil!");
    } else {
      Serial.println("SD kart başarıyla başlatıldı");
    }
  }
  
  // Harita belleğini dinamik olarak tahsis et
  haritayiIlklendir();
  
  // Başlangıç harita dosyasını oluştur
  haritayiKaydet();
  
  Serial.println("ESP32-CAM Çizgi Takibi ve Haritalama Sistemi hazır!");
  delay(1000);
}

void loop() {
  unsigned long simdikiZaman = millis();
  
  // Çizgi takibi için kameradan görüntü al
  if (simdikiZaman - sonKomutZamani >= komutAraligi) {
    sonKomutZamani = simdikiZaman;
    
    // Görüntü al ve çizgi algılama
    cizgiBulundu = cizgiAlgila();
    
    // Çizgi algılama sonucuna göre PID kontrollü komut belirle
    char komut = cizgiKomutunuBelirle();
    
    // Komut stabilizasyonu (aynı komut en az 2 kez tekrarlanırsa gönder)
    if (komut == stabilKomut) {
      komutSayaci++;
    } else {
      komutSayaci = 0;
      stabilKomut = komut;
    }
    
    // Komut değişimi kontrolü ve gönderme
    if (komutSayaci >= 1 || simdikiZaman - sonKonumGuncellemeZamani > 500) {
      // Komutu Arduino'ya gönder
      Serial.write(komut);
      
      // Son komutu sakla
      if (komut != sonKomut) {
        Serial.print("Komut gönderildi: ");
        Serial.println(komut);
        sonKomut = komut;
      }
      
      // Konumu güncelle
      konumuGuncelle(komut);
      sonKonumGuncellemeZamani = simdikiZaman;
    }
    
    // Harita için mevcut konumu güncelle
    haritayiGuncelle();
  }
  
  // Belirli aralıklarla haritayı SD karta kaydet
  if (simdikiZaman - sonKayitZamani >= kayitAraligi) {
    sonKayitZamani = simdikiZaman;
    haritayiKaydet();
  }
  
  // Belirli aralıklarla fotoğraf çek (ilerleme görüntüsü)
  if (simdikiZaman - sonFotografZamani >= fotografAraligi) {
    sonFotografZamani = simdikiZaman;
    fotografCek();
  }
}

// Harita belleğini ilklendirme fonksiyonu
void haritayiIlklendir() {
  // Hafızada yeterli alan aç
  harita = (uint8_t**)malloc(HARITA_BOYUTU * sizeof(uint8_t*));
  
  if (harita == NULL) {
    Serial.println("Harita için bellek ayrılamadı!");
    return;
  }
  
  for (int i = 0; i < HARITA_BOYUTU; i++) {
    harita[i] = (uint8_t*)malloc(HARITA_BOYUTU * sizeof(uint8_t));
    
    if (harita[i] == NULL) {
      Serial.println("Harita satırı için bellek ayrılamadı!");
      return;
    }
    
    // Başlangıçta tüm haritayı sıfırla
    for (int j = 0; j < HARITA_BOYUTU; j++) {
      harita[i][j] = 0; // 0: Bilinmiyor, 1: Robot yolu, 2: Engel
    }
  }
  
  // Başlangıç pozisyonunu işaretle
  int merkez_x = HARITA_MERKEZI;
  int merkez_y = HARITA_MERKEZI;
  harita[merkez_y][merkez_x] = 1; // Başlangıç noktası
  
  Serial.println("Harita belleği başarıyla ilklendirildi");
}

// İyileştirilmiş çizgi algılama
bool cizgiAlgila() {
  // Kameradan görüntü al
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Kamera görüntüsü alınamadı!");
    return false;
  }
  
  // Görüntünün boyutlarını al
  int width = fb->width;
  int height = fb->height;
  uint8_t *buf = fb->buf;
  
  // Görüntünün alt 1/3'lük kısmına odaklan (çizgi genelde burada görünür)
  int taramaBaslangicY = height * 2 / 3;
  int taramaBitisY = height - 1;
  
  // Çizgi profili için değişkenler
  int enIyiSiyahBaslangic = -1;
  int enIyiSiyahBitis = -1;
  int enIyiSatir = -1;
  int enIyiGenislik = 0;
  
  // Her satır için en iyi çizgiyi bul
  uint8_t esikDegeri = 80; // Siyah için eşik değeri (düşük değerler daha koyu)
  
  // Birden fazla satırı tara ve en iyi çizgiyi bul
  for (int y = taramaBaslangicY; y <= taramaBitisY; y++) {
    // Satır içinde siyah bölgeleri ara
    int siyahBaslangic = -1;
    int siyahBitis = -1;
    
    for (int x = 0; x < width; x++) {
      uint8_t pikselDegeri = buf[y * width + x];
      
      // Siyah piksel mi?
      if (pikselDegeri < esikDegeri) {
        // Siyah bölge başlangıcı
        if (siyahBaslangic == -1) {
          siyahBaslangic = x;
        }
      } else {
        // Siyah bölge sonu
        if (siyahBaslangic != -1 && siyahBitis == -1) {
          siyahBitis = x - 1;
          
          // Siyah bölge genişliği
          int genislik = siyahBitis - siyahBaslangic + 1;
          
          // Geçerli bir çizgi mi? (minimum-maksimum genişlik kontrolü)
          if (genislik >= 5 && genislik <= width / 4) {
            // Şimdiye kadar bulunan en iyi çizgiden daha iyi mi?
            if (genislik > enIyiGenislik) {
              enIyiSiyahBaslangic = siyahBaslangic;
              enIyiSiyahBitis = siyahBitis;
              enIyiSatir = y;
              enIyiGenislik = genislik;
            }
          }
          
          // Bu bölgeyi temizle ve yeni siyah bölge ara
          siyahBaslangic = -1;
          siyahBitis = -1;
        }
      }
    }
    
    // Satır sonu kontrolü
    if (siyahBaslangic != -1) {
      siyahBitis = width - 1;
      int genislik = siyahBitis - siyahBaslangic + 1;
      
      if (genislik >= 5 && genislik <= width / 4) {
        if (genislik > enIyiGenislik) {
          enIyiSiyahBaslangic = siyahBaslangic;
          enIyiSiyahBitis = siyahBitis;
          enIyiSatir = y;
          enIyiGenislik = genislik;
        }
      }
    }
  }
  
  // En iyi çizgi bulundu mu?
  if (enIyiGenislik > 0) {
    cizgiX = (enIyiSiyahBaslangic + enIyiSiyahBitis) / 2;
    cizgiGenislik = enIyiGenislik;
    
    Serial.print("Çizgi bulundu! Satır: ");
    Serial.print(enIyiSatir);
    Serial.print(", X: ");
    Serial.print(cizgiX);
    Serial.print(", Genişlik: ");
    Serial.println(cizgiGenislik);
    
    esp_camera_fb_return(fb);
    
    sonCizgiZamani = millis();
    return true;
  }
  
  // Çizgi bulunamadı - daha geniş arama yap
  // Tüm görüntüyü taramak yerine belirli aralıklarla satırları kontrol et
  for (int y = height / 2; y < height; y += 4) {
    int siyahBaslangic = -1;
    int siyahBitis = -1;
    
    for (int x = 0; x < width; x++) {
      uint8_t pikselDegeri = buf[y * width + x];
      
      if (pikselDegeri < esikDegeri) {
        if (siyahBaslangic == -1) {
          siyahBaslangic = x;
        }
      } else {
        if (siyahBaslangic != -1 && siyahBitis == -1) {
          siyahBitis = x - 1;
          int genislik = siyahBitis - siyahBaslangic + 1;
          
          if (genislik >= 5 && genislik <= width / 3) {
            cizgiX = (siyahBaslangic + siyahBitis) / 2;
            cizgiGenislik = genislik;
            
            Serial.print("Çizgi bulundu (geniş tarama)! Satır: ");
            Serial.print(y);
            Serial.print(", X: ");
            Serial.print(cizgiX);
            Serial.print(", Genişlik: ");
            Serial.println(cizgiGenislik);
            
            esp_camera_fb_return(fb);
            
            sonCizgiZamani = millis();
            return true;
          }
          
          siyahBaslangic = -1;
          siyahBitis = -1;
        }
      }
    }
    
    // Satır sonu kontrolü
    if (siyahBaslangic != -1) {
      siyahBitis = width - 1;
      int genislik = siyahBitis - siyahBaslangic + 1;
      
      if (genislik >= 5 && genislik <= width / 3) {
        cizgiX = (siyahBaslangic + siyahBitis) / 2;
        cizgiGenislik = genislik;
        
        Serial.print("Çizgi bulundu (geniş tarama satır sonu)! Satır: ");
        Serial.print(y);
        Serial.print(", X: ");
        Serial.print(cizgiX);
        Serial.print(", Genişlik: ");
        Serial.println(cizgiGenislik);
        
        esp_camera_fb_return(fb);
        
        sonCizgiZamani = millis();
        return true;
      }
    }
  }
  
  // Çizgi bulunamadı
  Serial.println("Çizgi bulunamadı");
  esp_camera_fb_return(fb);
  
  // Son çizgi algılama zamanından bu yana 3 saniyeden az geçtiyse
  // son bilinen konumu kullanmaya devam et
  if (millis() - sonCizgiZamani < 3000 && cizgiBulundu) {
    Serial.println("Son bilinen çizgi konumu kullanılıyor");
    return true;
  }
  
  return false;
}

// PID kontrollü çizgi takibi
char cizgiKomutunuBelirle() {
  // Çizgi bulunamadıysa dur
  if (!cizgiBulundu) {
    kumulatifSapma = 0; // İntegral terimini sıfırla
    oncekiSapma = 0;    // Türev terimini sıfırla
    return CMD_DUR;
  }
  
  // Görüntü genişliği
  int screenWidth = 320; // QVGA genişliği
  
  // Görüntünün orta noktası
  int screenCenter = screenWidth / 2;
  
  // Çizginin orta noktadan sapma miktarı
  int sapma = cizgiX - screenCenter;
  
  // PID kontrol
  // P - Orantısal kontrol (anlık sapma)
  float p_out = P_katsayi * sapma;
  
  // I - İntegral kontrol (kümülatif sapma)
  kumulatifSapma = kumulatifSapma * 0.8 + sapma; // Entegral birikimi sınırla (0.8 katsayı)
  float i_out = I_katsayi * kumulatifSapma;
  
  // D - Türevsel kontrol (sapma değişim hızı)
  float d_out = D_katsayi * (sapma - oncekiSapma);
  oncekiSapma = sapma;
  
  // Toplam kontrol sinyali
  float pid_out = p_out + i_out + d_out;
  
  Serial.print("Çizgi sapması: ");
  Serial.print(sapma);
  Serial.print(", PID çıkışı: ");
  Serial.println(pid_out);
  
  // PID çıkışına göre komut belirle
  if (abs(pid_out) < 15) {
    // Çizgi ortada, ileri git
    return CMD_ILERI;
  } else if (pid_out < -60) {
    // Çizgi solda, sola hızlı dön
    return CMD_SOL;
  } else if (pid_out > 60) {
    // Çizgi sağda, sağa hızlı dön
    return CMD_SAG;
  } else if (pid_out < 0) {
    // Çizgi hafif solda, yavaşça sola dön
    // Burada kontrol sinyalinin büyüklüğüne göre ekstra komutlar eklenebilir
    return CMD_SOL;
  } else {
    // Çizgi hafif sağda, yavaşça sağa dön
    return CMD_SAG;
  }
}

// Gelişmiş konum güncelleme fonksiyonu
void konumuGuncelle(char komut) {
  // Zaman aralığını hesapla
  unsigned long simdikiZaman = millis();
  float deltaT = (simdikiZaman - sonKonumGuncellemeZamani) / 1000.0; // saniye cinsinden
  
  if (deltaT <= 0) deltaT = 0.1; // Sıfır bölünmesini önle
  
  // Önceki hızları sakla
  float oncekiVx = v_x;
  float oncekiVy = v_y;
  
  // Yönü derece cinsinden normalize et
  while (yon < 0) yon += 360.0;
  while (yon >= 360.0) yon -= 360.0;
  
  // Yönü radyan cinsine çevir
  float yonRadyan = yon * PI / 180.0;
  
  // Hareketleri komuta göre güncelle
  float hareketMesafesi = 2.0;  // Adım başına mesafe (cm)
  
  switch(komut) {
    case CMD_ILERI:
      // İleri hareket - yönü değiştirmeden ilerle
      v_x = hareketMesafesi * sin(yonRadyan) / deltaT;
      v_y = hareketMesafesi * cos(yonRadyan) / deltaT;
      break;
    case CMD_GERI:
      // Geri hareket - ters yönde hareket
      v_x = -hareketMesafesi * sin(yonRadyan) / deltaT;
      v_y = -hareketMesafesi * cos(yonRadyan) / deltaT;
      break;
    case CMD_SOL:
      // Sola dönüş - yönü güncelle ve dönüş hızını hesapla
      yon -= 3.0;  // 3 derece dön (daha küçük adımlar)
      // Dönüş sırasında hafif ilerleme
      v_x = 0.5 * sin(yonRadyan) / deltaT;
      v_y = 0.5 * cos(yonRadyan) / deltaT;
      break;
    case CMD_SAG:
      // Sağa dönüş - yönü güncelle ve dönüş hızını hesapla
      yon += 3.0;  // 3 derece dön (daha küçük adımlar)
      // Dönüş sırasında hafif ilerleme
      v_x = 0.5 * sin(yonRadyan) / deltaT;
      v_y = 0.5 * cos(yonRadyan) / deltaT;
      break;
    case CMD_DUR:
      // Durma durumunda hızı azalt ama tamamen sıfırlama (daha doğal haritalama için)
      v_x *= 0.8;
      v_y *= 0.8;
      break;
  }
  
  // Hızı yumuşat (ani değişimleri önle)
  v_x = 0.7 * v_x + 0.3 * oncekiVx;
  v_y = 0.7 * v_y + 0.3 * oncekiVy;
  
  // Konumu güncelle (hız x zaman)
  x_pos += v_x * deltaT;
  y_pos += v_y * deltaT;
  
  // Konumu yakala (hata birikimini önle)
  konum_guven *= 0.99; // Her adımda güven faktörünü biraz azalt
  
  // Her 100 adımda bir konumu sıfırla (tamamen yanlış konumdan kaçınmak için)
  static int adimSayaci = 0;
  adimSayaci++;
  
  if (adimSayaci >= 1000) {
    adimSayaci = 0;
    konum_guven = 1.0; // Konuma güveni yenile
  }
}

// Gelişmiş harita güncelleme fonksiyonu
void haritayiGuncelle() {
  // Robot pozisyonunu harita koordinatlarına dönüştür (merkez nokta kaydırmalı)
  int harita_x = (int)(x_pos / HUCRE_BOYUTU) + HARITA_MERKEZI;
  int harita_y = (int)(y_pos / HUCRE_BOYUTU) + HARITA_MERKEZI;
  
  // Harita sınırlarını kontrol et
  if (harita_x >= 0 && harita_x < HARITA_BOYUTU && 
      harita_y >= 0 && harita_y < HARITA_BOYUTU) {
    
    // Robotun bulunduğu konumu işaretle
    harita[harita_y][harita_x] = 1; // Robot yolu
    
    // Robotun yönündeki birkaç adım için de işaretle (daha düzgün bir harita oluşturmak için)
    float yonRadyan = yon * PI / 180.0;
    
    for (int i = 1; i <= 2; i++) {
      int ileri_x = harita_x + (int)(i * sin(yonRadyan));
      int ileri_y = harita_y + (int)(i * cos(yonRadyan));
      
      if (ileri_x >= 0 && ileri_x < HARITA_BOYUTU && 
          ileri_y >= 0 && ileri_y < HARITA_BOYUTU) {
        // Eğer bu hücre daha önce işaretlenmediyse işaretle
        if (harita[ileri_y][ileri_x] == 0) {
          harita[ileri_y][ileri_x] = 1;
        }
      }
    }
    
    // Robotun çevresini de işaretle (daha kalın çizgi oluşturmak için)
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        int komsu_x = harita_x + dx;
        int komsu_y = harita_y + dy;
        
        if (komsu_x >= 0 && komsu_x < HARITA_BOYUTU && 
            komsu_y >= 0 && komsu_y < HARITA_BOYUTU) {
          // Eğer bu hücre daha önce işaretlenmediyse işaretle
          if (harita[komsu_y][komsu_x] == 0) {
            harita[komsu_y][komsu_x] = 1;
          }
        }
      }
    }
  }
}

// Gelişmiş harita kaydetme fonksiyonu
void haritayiKaydet() {
  // SD kartın başlatılıp başlatılmadığını kontrol et
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("SD kart yok, harita kaydedilemiyor");
    return;
  }

  // TXT formatında dosya oluştur (insan tarafından okunabilir formatta)
  String filename = "/harita_" + String(millis()) + ".txt";
  File file = SD_MMC.open(filename, FILE_WRITE);
  
  if (!file) {
    Serial.println("Dosya açılamadı!");
    return;
  }
  
  // Mevcut konum etrafındaki alanı kaydet (tüm haritayı kaydetmek yerine)
  int merkez_x = (int)(x_pos / HUCRE_BOYUTU) + HARITA_MERKEZI;
  int merkez_y = (int)(y_pos / HUCRE_BOYUTU) + HARITA_MERKEZI;
  int goruntuYaricap = 50; // Görüntülenen alan yarıçapı
  
  // Harita üstbilgisi
  file.println("ESP32-CAM Çizgi Takip Robotu - Harita Görüntüsü");
  file.println("==============================================");
  file.print("Zaman: ");
  file.print(millis() / 1000);
  file.println(" sn");
  file.print("Konum: (");
  file.print(x_pos);
  file.print(", ");
  file.print(y_pos);
  file.println(") cm");
  file.print("Yön: ");
  file.print(yon);
  file.println(" derece");
  file.println("==============================================");
  file.println();
  
  // Harita sembollerini açıkla
  file.println("Semboller: ' ' = Bilinmiyor, '#' = Robot Yolu, 'X' = Mevcut Konum, '*' = Engel");
  file.println();
  
  // Görüntülenen bölgenin sınırları
  int baslangic_x = max(0, merkez_x - goruntuYaricap);
  int bitis_x = min(HARITA_BOYUTU - 1, merkez_x + goruntuYaricap);
  int baslangic_y = max(0, merkez_y - goruntuYaricap);
  int bitis_y = min(HARITA_BOYUTU - 1, merkez_y + goruntuYaricap);
  
  // Harita verilerini dosyaya yaz
  for (int y = baslangic_y; y <= bitis_y; y++) {
    for (int x = baslangic_x; x <= bitis_x; x++) {
      // Mevcut konum 'X' ile işaretleniyor
      if (x == merkez_x && y == merkez_y) {
        file.print("X");
      } 
      // Robot yolu
      else if (harita[y][x] == 1) {
        file.print("#");
      }
      // Engel
      else if (harita[y][x] == 2) {
        file.print("*");
      }
      // Bilinmiyor
      else {
        file.print(" ");
      }
    }
    file.println(); // Yeni satır
  }
  
  file.println();
  file.println("==============================================");
  file.println("Harita Boyutları: " + String(HARITA_BOYUTU) + "x" + String(HARITA_BOYUTU) + " hücre");
  file.println("Hücre Boyutu: " + String(HUCRE_BOYUTU) + "x" + String(HUCRE_BOYUTU) + " cm");
  file.close();
  
  Serial.println("Harita görüntüsü kaydedildi: " + filename);
  
  // Tam haritayı CSV formatında kaydet (bilgisayarda işlemek için)
  filename = "/harita_data_" + String(millis()) + ".csv";
  file = SD_MMC.open(filename, FILE_WRITE);
  
  if (!file) {
    Serial.println("CSV dosyası açılamadı!");
    return;
  }
  
  // CSV başlığı
  file.println("x,y,durum,aciklama");
  
  // Harita verilerini CSV formatında yaz - sadece işaretli hücreler
  for (int y = 0; y < HARITA_BOYUTU; y++) {
    for (int x = 0; x < HARITA_BOYUTU; x++) {
      if (harita[y][x] > 0) {  // Sadece işaretlenmiş hücreleri kaydet (sıkıştırma)
        float gercek_x = (x - HARITA_MERKEZI) * HUCRE_BOYUTU;
        float gercek_y = (y - HARITA_MERKEZI) * HUCRE_BOYUTU;
        
        file.print(gercek_x);
        file.print(",");
        file.print(gercek_y);
        file.print(",");
        file.print(harita[y][x]);
        file.print(",");
        
        // Özel durumlar için açıklama ekle
        if (x == merkez_x && y == merkez_y) {
          file.println("mevcut_konum");
        } else if (harita[y][x] == 1) {
          file.println("yol");
        } else if (harita[y][x] == 2) {
          file.println("engel");
        } else {
          file.println("");
        }
      }
    }
  }
  
  file.close();
  Serial.println("CSV dosyası kaydedildi: " + filename);
  
  // Yeni JSON formatında gelişmiş harita metadata kaydı
  filename = "/harita_meta_" + String(millis()) + ".json";
  file = SD_MMC.open(filename, FILE_WRITE);
  
  if (!file) {
    Serial.println("JSON dosyası açılamadı!");
    return;
  }
  
  // JSON formatında metadata bilgilerini kaydet
  file.println("{");
  file.println("  \"metadata\": {");
  file.print("    \"zaman\": ");
  file.print(millis());
  file.println(",");
  file.print("    \"harita_boyutu\": ");
  file.print(HARITA_BOYUTU);
  file.println(",");
  file.print("    \"hucre_boyutu\": ");
  file.print(HUCRE_BOYUTU);
  file.println(",");
  file.print("    \"merkez_noktasi\": ");
  file.print(HARITA_MERKEZI);
  file.println(",");
  file.print("    \"toplam_hucre_sayisi\": ");
  
  // Toplam işaretli hücre sayısını hesapla
  int isaretiHucreSayisi = 0;
  for (int y = 0; y < HARITA_BOYUTU; y++) {
    for (int x = 0; x < HARITA_BOYUTU; x++) {
      if (harita[y][x] > 0) isaretiHucreSayisi++;
    }
  }
  
  file.print(isaretiHucreSayisi);
  file.println();
  file.println("  },");
  
  file.println("  \"robot\": {");
  file.print("    \"konum_x\": ");
  file.print(x_pos);
  file.println(",");
  file.print("    \"konum_y\": ");
  file.print(y_pos);
  file.println(",");
  file.print("    \"yon\": ");
  file.print(yon);
  file.println(",");
  file.print("    \"hiz_x\": ");
  file.print(v_x);
  file.println(",");
  file.print("    \"hiz_y\": ");
  file.print(v_y);
  file.println(",");
  file.print("    \"konum_guven\": ");
  file.print(konum_guven);
  file.println();
  file.println("  },");
  
  file.println("  \"cizgi_takibi\": {");
  file.print("    \"cizgi_bulundu\": ");
  file.print(cizgiBulundu ? "true" : "false");
  file.println(",");
  file.print("    \"cizgi_x\": ");
  file.print(cizgiX);
  file.println(",");
  file.print("    \"cizgi_genislik\": ");
  file.print(cizgiGenislik);
  file.println();
  file.println("  }");
  
  file.println("}");
  file.close();
  
  Serial.println("JSON metadata dosyası kaydedildi: " + filename);
}

// Geliştirilmiş fotoğraf çekme ve kaydetme fonksiyonu
void fotografCek() {
  // Fotoğraf için kamera modunu değiştir
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_pixformat(s, PIXFORMAT_JPEG); // Fotoğraf için JPEG formatına geç
  }
  
  // Fotoğraf çek
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Fotoğraf çekilemedi!");
    return;
  }
  
  // SD kartın başlatılıp başlatılmadığını kontrol et
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("SD kart yok, fotoğraf kaydedilemiyor");
    esp_camera_fb_return(fb);
    return;
  }
  
  // Klasör oluştur - tarih ve saat bilgisi ile
  String klasorAdi = "/goruntular";
  if (!SD_MMC.exists(klasorAdi)) {
    SD_MMC.mkdir(klasorAdi);
  }
  
  // Dosya adı oluştur - daha düzenli
  String dosyaAdi = String(millis());
  String filename = klasorAdi + "/foto_" + dosyaAdi + ".jpg";
  
  // Dosyayı aç
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Dosya açılamadı!");
    esp_camera_fb_return(fb);
    return;
  }
  
  // Fotoğrafı dosyaya yaz
  file.write(fb->buf, fb->len);
  file.close();
  
  // Fotoğraf belleğini serbest bırak
  esp_camera_fb_return(fb);
  
  Serial.println("Fotoğraf kaydedildi: " + filename);
  
  // JSON formatında detaylı metadata bilgisi kaydet
  String infoname = klasorAdi + "/foto_meta_" + dosyaAdi + ".json";
  file = SD_MMC.open(infoname, FILE_WRITE);
  
  if (file) {
    file.println("{");
    file.println("  \"goruntu\": {");
    file.print("    \"dosya\": \"");
    file.print(filename);
    file.println("\",");
    file.print("    \"zaman\": ");
    file.print(millis());
    file.println(",");
    file.print("    \"boyut\": ");
    file.print(fb->len);
    file.println(" ");
    file.println("  },");
    
    file.println("  \"robot\": {");
    file.print("    \"konum_x\": ");
    file.print(x_pos);
    file.println(",");
    file.print("    \"konum_y\": ");
    file.print(y_pos);
    file.println(",");
    file.print("    \"yon\": ");
    file.print(yon);
    file.println();
    file.println("  },");
    
    file.println("  \"cizgi_takibi\": {");
    file.print("    \"cizgi_bulundu\": ");
    file.print(cizgiBulundu ? "true" : "false");
    file.println();
    file.println("  }");
    
    file.println("}");
    file.close();
  }
  
  // Çizgi takibi için tekrar gri tonlamaya geri dön
  if (s) {
    s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
  }
}

// Hafıza temizleme fonksiyonu - program sonlandırılırken çağrılabilir
void temizle() {
  // Harita verisi için tahsis edilen belleği serbest bırak
  if (harita != NULL) {
    for (int i = 0; i < HARITA_BOYUTU; i++) {
      if (harita[i] != NULL) {
        free(harita[i]);
      }
    }
    free(harita);
    harita = NULL;
  }
}