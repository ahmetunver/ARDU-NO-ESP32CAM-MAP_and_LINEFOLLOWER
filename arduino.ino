/*
 * Arduino UNO - Ultrasonik Sensörlü Çizgi Takip Robot Kontrol Kodu (Gelişmiş Versiyon)
 * ESP32-CAM'den gelen çizgi takip komutlarını alır ve
 * ultrasonik sensörle engel algılama yaparak motor sürücüsünü kontrol eder.
 * 
 * Geliştirmeler:
 * - Daha dengeli ve minimal engel algılama
 * - Dar alanlara uygun küçük hareketler
 * - Daha verimli engellerden kaçınma stratejisi
 * - Gelişmiş filtreleme algoritması
 * - Çoklu uzaklık ölçümü doğrulama sistemi
 */

// Motor Kontrol Pinleri
#define ENA 5  // Sol motorlar hız kontrolü (PWM)
#define ENB 6  // Sağ motorlar hız kontrolü (PWM)
#define IN1 8  // Sol motorlar yön kontrol 1
#define IN2 9  // Sol motorlar yön kontrol 2
#define IN3 10 // Sağ motorlar yön kontrol 1
#define IN4 11 // Sağ motorlar yön kontrol 2

// Hareket Hızları
#define MAX_HIZ 200         // Maksimum hız (0-255 arası)
#define MIN_HIZ 80          // Minimum hız sınırı
#define ILERI_HIZ 170       // İleri hareket hızı
#define NORMAL_DONUS_HIZ 160 // Normal dönüş hızı
#define KESKIN_DONUS_HIZ 180 // Keskin dönüş hızı
#define DONUS_CARPAN 0.6    // Dönüşlerde iç tekerlek hız çarpanı (0-1 arası)
#define HIZ_IVME 15         // Hız değişim adımı (yumuşak geçişler için)
#define KACINMA_HIZ 160     // Engelden kaçınma hızı - orta seviyeli

// ESP32-CAM'den gelen komutlar
#define CMD_ILERI  'F'  // Forward
#define CMD_GERI   'B'  // Backward
#define CMD_SOL    'L'  // Left
#define CMD_SAG    'R'  // Right
#define CMD_DUR    'S'  // Stop

// Ultrasonik Sensör Pinleri
#define TRIG_PIN 2      // Ultrasonik sensör TRIG pini
#define ECHO_PIN 3      // Ultrasonik sensör ECHO pini

// Engel algılama parametreleri - Daha dengeli değerler
#define GUVENLI_MESAFE 30    // Güvenli mesafe 30cm (40cm'den düşürüldü)
#define ENGEL_MESAFESI 25    // Engel algılama mesafesi 25cm (35cm'den düşürüldü)
#define KRITIK_MESAFE 15     // Kritik mesafe - acil kaçınma (20cm'den düşürüldü)
#define MINIMUM_MESAFE 5     // Minimum mesafe (cm)

// Robot durumları
#define DURUM_NORMAL 0      // Normal çizgi takibi
#define DURUM_ENGEL_VAR 1   // Engel algılandı, kaçınma modunda
#define DURUM_KRITIK_ENGEL 2 // Kritik engel durumu (yeni)
#define DURUM_GECIS 3       // Geçiş modunda (değiştirildi)

// Hareket durumunu saklamak için değişken
char simdikiKomut = CMD_DUR;
char sonKomut = CMD_DUR;
unsigned long sonKomutZamani = 0;
const unsigned long komutZamanAsimi = 1500; // 1.5 saniye içinde komut gelmezse dur

// Yumuşak hareket için hız değişkenleri
int solMotorHedef = 0;
int sagMotorHedef = 0;
int solMotorSimdiki = 0;
int sagMotorSimdiki = 0;

// LED pin
#define LED_PIN 13 // Arduino üzerindeki dahili LED

// Robot durumu
int robotDurumu = DURUM_NORMAL;
unsigned long engelZamani = 0;
const unsigned long engelKacinmaZamani = 2000; // Engellerden kaçınma süresi (ms) - 3000'den 2000'e düşürüldü
unsigned long sonMesafeOlcumZamani = 0;
const unsigned long mesafeOlcumAraligi = 80; // 80ms'de bir mesafe ölç (50'den 80'e artırıldı)

// Engel kaçınma yönü
int kacinmaYonu = 0; // 0: Sağ, 1: Sol

// Filtre edilmiş mesafe değerleri - Daha dengeli filtreleme
float filtrelenmisUzaklik = 100.0;
float filtreKatsayi = 0.2; // Filtre katsayısı (0.3'ten 0.2'ye düşürüldü)

// Ortam öğrenme değişkenleri
#define HAREKET_KAYIT_BOYUTU 5
char sonHareketler[HAREKET_KAYIT_BOYUTU]; // Son hareketleri saklamak için dizi
int hareketIndeks = 0;

// Engel doğrulama için değişkenler (yeni eklendi)
#define DOGRULAMA_SAYISI 3
float sonUzakliklar[DOGRULAMA_SAYISI]; // Son birkaç uzaklık ölçümünü sakla
int uzaklikIndeks = 0;
bool engelDogrulandi = false;

// Engel kaçınma aşamaları (yeni) - Süreler kısaltıldı, daha minimal hareketler için
#define KACINMA_ASAMA_SAYISI 5
int kacinmaAsamasi = 0;
unsigned long asamaBaslangicZamani = 0;
const unsigned long asamaSureleri[KACINMA_ASAMA_SAYISI] = {400, 400, 600, 300, 300}; // Her aşamanın süresi (ms) - daha kısa süreler

// Ultrasonik sensör ile mesafe ölçümü - gelişmiş filtreli versiyon
float mesafeOlc() {
  // Ortalama için birden çok ölçüm yap
  float toplam = 0;
  int gecerliOlcumSayisi = 0;
  const int OLCUM_SAYISI = 3;
  
  for (int i = 0; i < OLCUM_SAYISI; i++) {
    // Sensörü tetikle
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Echo süresini ölç
    long sure = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms zaman aşımı
    
    // Mesafeyi hesapla (ses hızı 343m/s veya 0.0343cm/µs)
    float mesafe = sure * 0.0343 / 2;
    
    // Anormal değerleri filtrele
    if (mesafe <= 400 && mesafe > 0) {
      toplam += mesafe;
      gecerliOlcumSayisi++;
    }
    
    delayMicroseconds(10); // Ölçümler arası kısa bekleme
  }
  
  // Eğer geçerli ölçüm yoksa, varsayılan değer döndür
  if (gecerliOlcumSayisi == 0) {
    return 400; // Maksimum ölçüm mesafesi veya hata durumu
  }
  
  // Ortalama mesafeyi döndür
  return toplam / gecerliOlcumSayisi;
}

// Durum bilgisi yazdırma fonksiyonu (hata ayıklama için)
void durumYazdir() {
  Serial.print("Durum: ");
  switch (robotDurumu) {
    case DURUM_NORMAL:
      Serial.print("NORMAL");
      break;
    case DURUM_ENGEL_VAR:
      Serial.print("ENGEL VAR");
      break;
    case DURUM_KRITIK_ENGEL:
      Serial.print("KRITIK ENGEL");
      break;
    case DURUM_GECIS:
      Serial.print("GECIS");
      break;
  }
  
  Serial.print(", Mesafe: ");
  Serial.print(filtrelenmisUzaklik);
  Serial.print("cm, Komut: ");
  Serial.print(simdikiKomut);
  Serial.print(", Sol Motor: ");
  Serial.print(solMotorSimdiki);
  Serial.print(", Sağ Motor: ");
  Serial.println(sagMotorSimdiki);
}

// Engel algılama durumunu kontrol et (gelişmiş versiyon - daha minimal tepkiler)
void engelDurumunuKontrolEt() {
  // Mevcut durumu yazdır
  static unsigned long sonDurumYazdirmaZamani = 0;
  unsigned long simdikiZaman = millis();
  
  // Her 1 saniyede bir durum bilgisi yazdır
  if (simdikiZaman - sonDurumYazdirmaZamani > 1000) {
    sonDurumYazdirmaZamani = simdikiZaman;
    durumYazdir();
  }
  
  // Çok yakında kritik engel durumu - yavaş durdurma ve kısa geri manevra
  if (filtrelenmisUzaklik < 12 && filtrelenmisUzaklik > MINIMUM_MESAFE) {
    Serial.println("DİKKAT: Yakın engel algılandı - minimal kaçınma yapılıyor");
    yumusatHizDurdur(); // Yumuşak duruş
    delay(300);  // Kısa bir duraklama
    
    // Kısa ve yavaş geri manevra
    motorYon(-1, -1);
    hizAyarla(ILERI_HIZ * 0.6, ILERI_HIZ * 0.6); // Yarı hızla geri
    delay(500); // Kısa süre geri git
    
    // Hafif dönüş manevra
    if (random(2) == 0) {  // Rastgele yön seçimi
      motorYon(1, 1);  // Sağa hafif dönüş
      hizAyarla(ILERI_HIZ * 0.7, ILERI_HIZ * 0.5);
    } else {
      motorYon(1, 1);  // Sola hafif dönüş
      hizAyarla(ILERI_HIZ * 0.5, ILERI_HIZ * 0.7);
    }
    delay(400);
    
    // Yeniden kontrol et
    robotDurumu = DURUM_ENGEL_VAR;
  }
}

// Motorların kademeli hız güncellemesi
void hizlariGuncelle() {
  // Sol motor hızını kademeli güncelle
  if (solMotorSimdiki < solMotorHedef) {
    solMotorSimdiki = min(solMotorSimdiki + HIZ_IVME, solMotorHedef);
  } else if (solMotorSimdiki > solMotorHedef) {
    solMotorSimdiki = max(solMotorSimdiki - HIZ_IVME, solMotorHedef);
  }
  
  // Sağ motor hızını kademeli güncelle
  if (sagMotorSimdiki < sagMotorHedef) {
    sagMotorSimdiki = min(sagMotorSimdiki + HIZ_IVME, sagMotorHedef);
  } else if (sagMotorSimdiki > sagMotorHedef) {
    sagMotorSimdiki = max(sagMotorSimdiki - HIZ_IVME, sagMotorHedef);
  }
  
  // PWM değerlerini motorlara uygula
  analogWrite(ENA, solMotorSimdiki);
  analogWrite(ENB, sagMotorSimdiki);
}

// Basit motor testi
void motorTest() {
  Serial.println("Motor testi yapılıyor...");
  
  // İleri git
  Serial.println("İleri test...");
  motorYon(1, 1); // Her iki motor ileri yönde
  hizAyarla(100, 100); // Düşük hızda test
  delay(500);
  
  // Sola dön
  Serial.println("Sol dönüş test...");
  motorYon(1, 1); // Her iki motor ileri yönde
  hizAyarla(50, 100); // Sol motor daha yavaş
  delay(500);
  
  // Sağa dön
  Serial.println("Sağ dönüş test...");
  motorYon(1, 1); // Her iki motor ileri yönde
  hizAyarla(100, 50); // Sağ motor daha yavaş
  delay(500);
  
  // Keskin dönüş
  Serial.println("Keskin dönüş test...");
  motorYon(-1, 1); // Sol motor geri, sağ motor ileri
  hizAyarla(80, 80);
  delay(500);
  
  // Dur
  Serial.println("Durdurma test...");
  yumusatHizDurdur();
  delay(500);
  
  Serial.println("Motor testi tamamlandı.");
}

// Motor yönlerini ayarla
// dir değeri: 1=ileri, -1=geri, 0=serbest
void motorYon(int solDir, int sagDir) {
  // Sol motor yönü
  if (solDir == 1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (solDir == -1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  
  // Sağ motor yönü
  if (sagDir == 1) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (sagDir == -1) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

// Hız hedefini ayarla
void hizAyarla(int solHiz, int sagHiz) {
  // Hız sınırlaması
  solHiz = constrain(solHiz, 0, MAX_HIZ);
  sagHiz = constrain(sagHiz, 0, MAX_HIZ);
  
  // Hedef hızları güncelle
  solMotorHedef = solHiz;
  sagMotorHedef = sagHiz;
}

// Acil durma - hemen dur
void motorlariDurdur() {
  // Hemen durdurma için hızları sıfırla
  solMotorHedef = 0;
  sagMotorHedef = 0;
  solMotorSimdiki = 0;
  sagMotorSimdiki = 0;
  
  // PWM değerlerini doğrudan sıfırla
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Motor yönlerini serbest bırak
  motorYon(0, 0);
}

// Yumuşak durma
void yumusatHizDurdur() {
  // Hedef hızları sıfırla (kademeli yavaşlama için)
  solMotorHedef = 0;
  sagMotorHedef = 0;
  
  // Motor yönlerini serbest bırak
  motorYon(0, 0);
}

// Normal çizgi takibi modu
void normalMod() {
  // ESP32-CAM'den komut var mı diye kontrol et
  if (Serial.available() > 0) {
    char alinanKomut = Serial.read();
    
    // Sadece geçerli komutları kabul et
    if (alinanKomut == CMD_ILERI || alinanKomut == CMD_GERI || 
        alinanKomut == CMD_SOL || alinanKomut == CMD_SAG || 
        alinanKomut == CMD_DUR) {
      
      simdikiKomut = alinanKomut;
      sonKomutZamani = millis();
      
      // Son hareketi kaydet
      sonHareketler[hareketIndeks] = simdikiKomut;
      hareketIndeks = (hareketIndeks + 1) % HAREKET_KAYIT_BOYUTU;
      
      // Komut değiştiyse LED'i yakıp söndür ve bilgi ver
      if (simdikiKomut != sonKomut) {
        // LED ile komut alındığını göster
        digitalWrite(LED_PIN, HIGH);
        delay(10);
        digitalWrite(LED_PIN, LOW);
        
        Serial.print("Komut alındı: ");
        Serial.println(simdikiKomut);
        sonKomut = simdikiKomut;
      }
    }
  }
  
  // Belirli süre içinde komut gelmediyse dur
  unsigned long simdikiZaman = millis();
  if (simdikiZaman - sonKomutZamani > komutZamanAsimi) {
    if (simdikiKomut != CMD_DUR) {
      Serial.println("Komut zaman aşımı, motorlar durduruluyor");
      simdikiKomut = CMD_DUR;
    }
  }
  
  // Alınan komuta göre hareket et (yumuşak dönüş ve hız değişimi ile)
  switch (simdikiKomut) {
    case CMD_ILERI:
      motorYon(1, 1); // İleri yön
      hizAyarla(ILERI_HIZ, ILERI_HIZ); // Eşit hız
      break;
    
    case CMD_GERI:
      motorYon(-1, -1); // Geri yön
      hizAyarla(ILERI_HIZ, ILERI_HIZ); // Eşit hız
      break;
    
    case CMD_SOL:
      motorYon(1, 1); // Her iki motor ileri
      // Yumuşak dönüş: Sol motor daha yavaş
      hizAyarla(NORMAL_DONUS_HIZ * DONUS_CARPAN, NORMAL_DONUS_HIZ);
      break;
    
    case CMD_SAG:
      motorYon(1, 1); // Her iki motor ileri
      // Yumuşak dönüş: Sağ motor daha yavaş
      hizAyarla(NORMAL_DONUS_HIZ, NORMAL_DONUS_HIZ * DONUS_CARPAN);
      break;
    
    case CMD_DUR:
    default:
      yumusatHizDurdur();
      break;
  }
}

// Gelişmiş engelden kaçınma modu - aşamalı ve daha minimal hareketler
void gelismisEngeldenKacinma(unsigned long simdikiZaman, bool kritikEngel) {
  // Aşama süresini kontrol et ve gerekirse güncelle
  if (simdikiZaman - asamaBaslangicZamani >= asamaSureleri[kacinmaAsamasi]) {
    kacinmaAsamasi++;
    asamaBaslangicZamani = simdikiZaman;
    
    // Aşama numarası taşması - kaçınma tamamlandı
    if (kacinmaAsamasi >= KACINMA_ASAMA_SAYISI) {
      robotDurumu = DURUM_GECIS;
      Serial.println("Engelden kaçınma tamamlandı. Geçiş moduna geçiliyor.");
      return;
    }
    
    Serial.print("Kaçınma aşaması: ");
    Serial.println(kacinmaAsamasi);
  }
  
  // Engel kritik mi değil mi ona göre hız faktörü - kritik durumda bile daha düşük hız
  float hizFaktoru = kritikEngel ? 0.8 : 0.7;
  
  // Aşamalara göre hareket - daha minimal hareketler
  switch (kacinmaAsamasi) {
    case 0: // Kısa geri çekilme aşaması
      motorYon(-1, -1); // Geri git
      
      if (kacinmaYonu == 0) { // Sağa kaçınma için, geri giderken hafif sola doğru
        hizAyarla(KACINMA_HIZ * 0.6 * hizFaktoru, KACINMA_HIZ * 0.5 * hizFaktoru);
      } else { // Sola kaçınma için, geri giderken hafif sağa doğru
        hizAyarla(KACINMA_HIZ * 0.5 * hizFaktoru, KACINMA_HIZ * 0.6 * hizFaktoru);
      }
      break;
      
    case 1: // Minimal dönüş aşaması - çok hafif dönüş
      if (kacinmaYonu == 0) { // Sağa dön
        motorYon(1, 1); // İki motor da ileri - karşılıklı hız farkı ile dönüş
        hizAyarla(KESKIN_DONUS_HIZ * 0.7 * hizFaktoru, KESKIN_DONUS_HIZ * 0.4 * hizFaktoru);
      } else { // Sola dön
        motorYon(1, 1); // İki motor da ileri - karşılıklı hız farkı ile dönüş
        hizAyarla(KESKIN_DONUS_HIZ * 0.4 * hizFaktoru, KESKIN_DONUS_HIZ * 0.7 * hizFaktoru);
      }
      break;
      
    case 2: // Kısa ileri hareket - dönüş yönünde çok hafif eğimli
      motorYon(1, 1); // İleri git
      
      if (kacinmaYonu == 0) { // Sağa çok hafif meyilli ileri
        hizAyarla(ILERI_HIZ * 0.7 * hizFaktoru, ILERI_HIZ * 0.6 * hizFaktoru);
      } else { // Sola çok hafif meyilli ileri
        hizAyarla(ILERI_HIZ * 0.6 * hizFaktoru, ILERI_HIZ * 0.7 * hizFaktoru);
      }
      break;
      
    case 3: // Düzeltme hareketi - çok hafif ters tarafa dönüş
      motorYon(1, 1); // İleri git
      
      if (kacinmaYonu == 0) { // Sağa kaçınma ise çok hafif sola doğru
        hizAyarla(ILERI_HIZ * 0.6 * hizFaktoru, ILERI_HIZ * 0.7 * hizFaktoru);
      } else { // Sola kaçınma ise çok hafif sağa doğru
        hizAyarla(ILERI_HIZ * 0.7 * hizFaktoru, ILERI_HIZ * 0.6 * hizFaktoru);
      }
      break;
      
    case 4: // Düz ileri - çok yavaş, çizgi aramaya başla
      motorYon(1, 1); // Düz ileri
      hizAyarla(ILERI_HIZ * 0.6 * hizFaktoru, ILERI_HIZ * 0.6 * hizFaktoru);
      break;
  }
}

// Gelişmiş kaçınma yönü belirleme
int gelismisKacinmaYonunuBelirle() {
  // Son hareketlere bakarak yönü belirle
  int solSayisi = 0;
  int sagSayisi = 0;
  
  // Son 3 hareket ağırlıklı olarak incelenir (daha yakın geçmiş)
  for (int i = 0; i < HAREKET_KAYIT_BOYUTU; i++) {
    if (sonHareketler[i] == CMD_SOL) solSayisi++;
    if (sonHareketler[i] == CMD_SAG) sagSayisi++;
  }
  
  // Eğer son hareketlerde daha çok sola dönüş varsa, engelden sola kaçınmak daha mantıklı olabilir
  // Son hareketin getirdiği ivmeyi de hesaba katarak
  if (solSayisi > sagSayisi || sonHareketler[(hareketIndeks + HAREKET_KAYIT_BOYUTU - 1) % HAREKET_KAYIT_BOYUTU] == CMD_SOL) {
    return 1; // Sola kaçınma
  } else {
    return 0; // Sağa kaçınma
  }
}

// Gelişmiş çizgi arama modu
void gelismisCizgiArama() {
  // ESP32-CAM'den komut var mı diye kontrol et - öncelikli olarak
  while (Serial.available() > 0) {
    char alinanKomut = Serial.read();
    
    // Sadece geçerli komutları kabul et
    if (alinanKomut == CMD_ILERI || alinanKomut == CMD_GERI || 
        alinanKomut == CMD_SOL || alinanKomut == CMD_SAG || 
        alinanKomut == CMD_DUR) {
      
      simdikiKomut = alinanKomut;
      sonKomutZamani = millis();
      
      // Çizgi tekrar bulundu, normal moda dön
      robotDurumu = DURUM_NORMAL;
      Serial.println("Çizgi tekrar bulundu. Normal moda dönülüyor.");
      
      // Komuta göre harekete geç
      switch (simdikiKomut) {
        case CMD_ILERI:
          motorYon(1, 1);
          hizAyarla(ILERI_HIZ, ILERI_HIZ);
          break;
        case CMD_SOL:
          motorYon(1, 1);
          hizAyarla(NORMAL_DONUS_HIZ * DONUS_CARPAN, NORMAL_DONUS_HIZ);
          break;
        case CMD_SAG:
          motorYon(1, 1);
          hizAyarla(NORMAL_DONUS_HIZ, NORMAL_DONUS_HIZ * DONUS_CARPAN);
          break;
        case CMD_GERI:
          motorYon(-1, -1);
          hizAyarla(ILERI_HIZ, ILERI_HIZ);
          break;
        case CMD_DUR:
          yumusatHizDurdur();
          break;
      }
      
      return;
    }
  }
  
  // Çizgi bulunamadıysa, daha geniş ve etkili sağ-sol yaparak ilerle
  static unsigned long aramaSuresi = 0;
  static int aramaYonu = 0; // 0: Sağ, 1: Sol
  static int aramaAsamasi = 0; // 0: Hafif dönüş, 1: Normal dönüş, 2: Keskin dönüş
  
  unsigned long simdikiZaman = millis();
  
  // Daha minimal bir zigzag arama stratejisi - iki aşamalı
  if (simdikiZaman - aramaSuresi > 600) { // Daha kısa arama süresi
    aramaSuresi = simdikiZaman;
    aramaYonu = 1 - aramaYonu; // Yönü değiştir
    
    // Sadece iki aşama kullan
    aramaAsamasi = (aramaAsamasi + 1) % 2;
    
    // Aşama değişimini bildirme
    Serial.print("Çizgi arama: Yön=");
    Serial.print(aramaYonu == 0 ? "Sağ" : "Sol");
    Serial.print(", Aşama=");
    Serial.println(aramaAsamasi);
  }
  
  // Daha minimal zigzag hareket stratejisi
  switch (aramaAsamasi) {
    case 0: // Çok hafif dönüşlü arama
      motorYon(1, 1); // İleri
      if (aramaYonu == 0) {
        // Sağa çok hafif dön
        hizAyarla(ILERI_HIZ * 0.7, ILERI_HIZ * 0.5);
      } else {
        // Sola çok hafif dön
        hizAyarla(ILERI_HIZ * 0.5, ILERI_HIZ * 0.7);
      }
      break;
      
    case 1: // Hafif dönüşlü arama - biraz daha belirgin
      motorYon(1, 1); // İleri
      if (aramaYonu == 0) {
        // Sağa hafif dön
        hizAyarla(ILERI_HIZ * 0.8, ILERI_HIZ * 0.6);
      } else {
        // Sola hafif dön
        hizAyarla(ILERI_HIZ * 0.6, ILERI_HIZ * 0.8);
      }
      break;
  }
}

// Hareket yardımcı fonksiyonları
void ileriGit(float hizOrani = 1.0) {
  motorYon(1, 1);
  hizAyarla(ILERI_HIZ * hizOrani, ILERI_HIZ * hizOrani);
}

void geriGit(float hizOrani = 1.0) {
  motorYon(-1, -1);
  hizAyarla(ILERI_HIZ * hizOrani, ILERI_HIZ * hizOrani);
}

void solaDon(float hizOrani = 1.0, bool keskin = false) {
  if (keskin) {
    motorYon(-1, 1);
    hizAyarla(KESKIN_DONUS_HIZ * hizOrani, KESKIN_DONUS_HIZ * hizOrani);
  } else {
    motorYon(1, 1);
    hizAyarla(NORMAL_DONUS_HIZ * DONUS_CARPAN * hizOrani, NORMAL_DONUS_HIZ * hizOrani);
  }
}

void sagaDon(float hizOrani = 1.0, bool keskin = false) {
  if (keskin) {
    motorYon(1, -1);
    hizAyarla(KESKIN_DONUS_HIZ * hizOrani, KESKIN_DONUS_HIZ * hizOrani);
  } else {
    motorYon(1, 1);
    hizAyarla(NORMAL_DONUS_HIZ * hizOrani, NORMAL_DONUS_HIZ * DONUS_CARPAN * hizOrani);
  }
}

void keskinSolaDon(float hizOrani = 1.0) {
  solaDon(hizOrani, true);
}

void keskinSagaDon(float hizOrani = 1.0) {
  sagaDon(hizOrani, true);
}

void setup() {
  // Motor kontrol pinlerini çıkış olarak ayarla
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Ultrasonik sensör pinlerini ayarla
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // LED pini çıkış olarak ayarla
  pinMode(LED_PIN, OUTPUT);
  
  // Serial iletişimi başlat (ESP32-CAM ile haberleşme için)
  Serial.begin(9600);
  
  // Başlangıçta motorları durdur
  motorlariDurdur();
  
  // Hareket kaydını temizle
  for (int i = 0; i < HAREKET_KAYIT_BOYUTU; i++) {
    sonHareketler[i] = CMD_DUR;
  }
  
  // Uzaklık dizisini başlat
  for (int i = 0; i < DOGRULAMA_SAYISI; i++) {
    sonUzakliklar[i] = 100.0; // Başlangıçta yüksek değer
  }
  
  // Başlangıç sinyali
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("Arduino hazır - Gelişmiş ultrasonik sensörlü çizgi takip motoru başlatıldı");
  
  // Kısa motor testi
  motorTest();
  
  Serial.println("Arduino başlatıldı, ESP32-CAM'den komut bekleniyor.");
}

void loop() {
  unsigned long simdikiZaman = millis();
  
  // Düzenli aralıklarla mesafe ölçümü yap
  if (simdikiZaman - sonMesafeOlcumZamani >= mesafeOlcumAraligi) {
    sonMesafeOlcumZamani = simdikiZaman;
    
    // Mesafe ölçümü yap
    float mesafe = mesafeOlc();
    
    // Filtrelenmiş mesafe hesapla (yumuşatma)
    filtrelenmisUzaklik = filtrelenmisUzaklik * (1.0 - filtreKatsayi) + mesafe * filtreKatsayi;
    
    // Ölçümü doğrulama dizisine ekle
    sonUzakliklar[uzaklikIndeks] = mesafe;
    uzaklikIndeks = (uzaklikIndeks + 1) % DOGRULAMA_SAYISI;
    
    // Engel doğrulama kontrolü - en az 2 ölçüm ENGEL_MESAFESI'nden küçükse
    engelDogrulandi = false;
    int engelSayaci = 0;
    for (int i = 0; i < DOGRULAMA_SAYISI; i++) {
      if (sonUzakliklar[i] < ENGEL_MESAFESI && sonUzakliklar[i] > MINIMUM_MESAFE) {
        engelSayaci++;
      }
    }
    
    // En az 2 ölçüm engel gösteriyorsa doğrula
    if (engelSayaci >= 2) {
      engelDogrulandi = true;
    }
    
    // Engel durumunu kontrol et - güncellenmiş
    if (robotDurumu == DURUM_NORMAL) {
      // Normal moddayken engel kontrolü
      if (filtrelenmisUzaklik < KRITIK_MESAFE && filtrelenmisUzaklik > MINIMUM_MESAFE && engelDogrulandi) {
        // Kritik mesafede engel algılandı, acil kaçınma moduna geç
        robotDurumu = DURUM_KRITIK_ENGEL;
        engelZamani = simdikiZaman;
        kacinmaAsamasi = 0;
        asamaBaslangicZamani = simdikiZaman;
        
        // Kaçınma yönünü belirle - daha gelişmiş yöntem
        kacinmaYonu = gelismisKacinmaYonunuBelirle();
        
        Serial.print("KRİTİK ENGEL ALGILANDI! Mesafe: ");
        Serial.print(filtrelenmisUzaklik);
        Serial.print(" cm. Acil kaçınma yönü: ");
        Serial.println(kacinmaYonu == 0 ? "Sağ" : "Sol");
        
        // Hemen dur
        motorlariDurdur();
      }
      else if (filtrelenmisUzaklik < ENGEL_MESAFESI && filtrelenmisUzaklik > MINIMUM_MESAFE && engelDogrulandi) {
        // Normal engel algılandı, engelden kaçınma moduna geç
        robotDurumu = DURUM_ENGEL_VAR;
        engelZamani = simdikiZaman;
        kacinmaAsamasi = 0;
        asamaBaslangicZamani = simdikiZaman;
        
        // Kaçınma yönünü belirle
        kacinmaYonu = gelismisKacinmaYonunuBelirle();
        
        Serial.print("Engel algılandı! Mesafe: ");
        Serial.print(filtrelenmisUzaklik);
        Serial.print(" cm. Engelden kaçınma yönü: ");
        Serial.println(kacinmaYonu == 0 ? "Sağ" : "Sol");
        
        // Yumuşak dur
        yumusatHizDurdur();
      }
    }
    else if ((robotDurumu == DURUM_ENGEL_VAR || robotDurumu == DURUM_KRITIK_ENGEL) && 
             filtrelenmisUzaklik >= GUVENLI_MESAFE) {
      // Kaçınma modundayken güvenli mesafeye ulaşıldı mı?
      robotDurumu = DURUM_GECIS;
      Serial.print("Güvenli mesafeye ulaşıldı. Mesafe: ");
      Serial.print(filtrelenmisUzaklik);
      Serial.println(" cm. Geçiş moduna geçiliyor.");
    }
  }
  
  // Aşırı yakın engeller için acil kontrol
  if (robotDurumu == DURUM_NORMAL || robotDurumu == DURUM_GECIS) {
    engelDurumunuKontrolEt();
  }
  
  // Robot durumuna göre hareket et
  if (robotDurumu == DURUM_NORMAL) {
    // Normal çizgi takibi modu
    normalMod();
  }
  else if (robotDurumu == DURUM_ENGEL_VAR) {
    // Engelden normal kaçınma modu
    gelismisEngeldenKacinma(simdikiZaman, false);
  }
  else if (robotDurumu == DURUM_KRITIK_ENGEL) {
    // Kritik engelden kaçınma modu - daha agresif
    gelismisEngeldenKacinma(simdikiZaman, true);
  }
  else if (robotDurumu == DURUM_GECIS) {
    // Geçiş modu - çizgiyi tekrar bulma
    gelismisCizgiArama();
  }
  
  // Hız güncellemesi - her döngüde yumuşak hız değişimi
  hizlariGuncelle();
  
  // Kısa bir gecikme
  delay(10);
}