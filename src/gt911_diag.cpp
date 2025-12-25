// Minidiagnóstico deshabilitado temporalmente
#if 0
#include <Arduino.h>
#include <Wire.h>

#define PIN_SDA 33
#define PIN_SCL 32
#define PIN_INT 18
#define I2C_ADDR 0x5D

TwoWire tw = TwoWire(1);

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_INT, INPUT_PULLUP);
  tw.begin(PIN_SDA, PIN_SCL, 400000);
  delay(50);
  Serial.println("Minidiagnostico GT911 - I2C productID y INT");
  // Leer ProductID 0x8140..0x8147
  uint16_t reg = 0x8140;
  tw.beginTransmission(I2C_ADDR);
  tw.write((uint8_t)(reg >> 8));
  tw.write((uint8_t)(reg & 0xFF));
  if (tw.endTransmission(false) != 0) {
    Serial.println("No se pudo iniciar lectura ProductID (endTransmission)");
  } else {
    if (tw.requestFrom(I2C_ADDR, (uint8_t)8) == 8) {
      Serial.print("ProductID: ");
      for (int i=0;i<8;i++) Serial.print((char)tw.read());
      Serial.println();
    } else {
      Serial.println("Lectura ProductID fallo (bytes insuf).");
    }
  }
  Serial.printf("Estado INT inicial: %s\n", digitalRead(PIN_INT)==LOW ? "LOW" : "HIGH");
}

int lastInt = HIGH;
unsigned long lastPrint = 0;

void loop() {
  int s = digitalRead(PIN_INT);
  if (s != lastInt) {
    Serial.printf("Cambio INT: %s -> %s\n", lastInt==LOW?"LOW":"HIGH", s==LOW?"LOW":"HIGH");
    lastInt = s;
  }
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    // intenta una simple transacción I2C ping
    tw.beginTransmission(I2C_ADDR);
    int err = tw.endTransmission();
    Serial.printf("Ping I2C 0x%02X err=%d\n", I2C_ADDR, err);
  }
  delay(10);
}
#endif
