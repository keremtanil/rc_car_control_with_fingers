// Pin tanımları
const int rightMotorForward = 2; // Sağ motor ileri yön
const int rightMotorBackward = 3; // Sağ motor geri yön
const int leftMotorForward = 5; // Sol motor ileri yön
const int leftMotorBackward = 6; // Sol motor geri yön

void setup() {
  // Motor pinlerini çıkış olarak ayarla
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  
  // Seri haberleşmeyi başlat
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    // Gelen komutları değerlendir
    switch (command) {
      case '0': // Durma
        dur();
        break;
      case '1': // Geri
        geri();
        break;
      case '2': // Sağa
        saga();
        break;
      case '3': // Sola
        sola();
        break;
      case '4': // İleri
      case '5': // İleri (4-5 aynı işlev)
        ileri();
        break;
    }
  }
}

// Motor hareket fonksiyonları
void dur() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  Serial.println("Motors Stopped");
}

void ileri() {
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  Serial.println("Moving Forward");
}

void geri() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  Serial.println("Moving Backward");
}

void saga() {
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  Serial.println("Moving Right");
}

void sola() {
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  Serial.println("Moving Left");
}
