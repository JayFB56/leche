#if 0
#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Wire.h>

#define PIN_SDA 33
#define PIN_SCL 32
#define PIN_INT 18
#define I2C_ADDR 0x5D

// Pines (ajusta si tu placa usa otros)
#define PIN_BACKLIGHT 27

class LGFX_ESP32_3248S035C : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488     _panel_instance;
  lgfx::Bus_SPI           _bus_instance;
  lgfx::Touch_GT911       _touch_instance;

public:
  LGFX_ESP32_3248S035C(void) {
    { // Configuración Bus SPI
      auto cfg = _bus_instance.config();
      cfg.spi_host = VSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // Configuración Panel
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 15;
      cfg.pin_rst          = -1;
      cfg.panel_width      = 320;
      cfg.panel_height     = 480;
      cfg.bus_shared       = true;
      cfg.offset_rotation  = 0;
      _panel_instance.config(cfg);
    }

    { // Configuración táctil GT911
      auto cfg = _touch_instance.config();
      cfg.x_min      = 0;
      cfg.x_max      = 319;
      cfg.y_min      = 0;
      cfg.y_max      = 479;
      cfg.pin_int    = PIN_INT;
      cfg.bus_shared = false;
      cfg.i2c_port   = 1;
      cfg.i2c_addr   = 0x5D;
      cfg.pin_sda    = PIN_SDA;
      cfg.pin_scl    = PIN_SCL;
      cfg.freq       = 400000;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);

    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);
  }
};

LGFX_ESP32_3248S035C tft;
TwoWire tw = TwoWire(1);

void i2cScan(TwoWire &w) {
  Serial.println("Escaneando bus I2C en busca de dispositivos...");
  for (uint8_t addr = 1; addr < 127; ++addr) {
    w.beginTransmission(addr);
    uint8_t err = w.endTransmission();
    if (err == 0) {
      Serial.printf("  Dispositivo en 0x%02X\n", addr);
      delay(10);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Test de touch GT911 para ESP32 3248S035C ---");

  // Inicializamos TwoWire en el mismo bus configurado para el touch
  tw.begin(PIN_SDA, PIN_SCL, 400000);

  Serial.println("Iniciando pantalla (LovyanGFX)...");
  tft.init();

  // Probamos varias rotaciones que suelen necesitarse en estas placas
  for (int r : {0, 1, 2, 3, 4, 5, 6, 7}) {
    tft.setRotation(r);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.drawString(String("Rot=") + r, tft.width()/2, 20, 1);
    delay(200);
  }

  // Pantalla inicial
  tft.setRotation(5); // mantiene la sugerida por tu código
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(3);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("TEST TOUCH GT911", tft.width()/2, 24);

  // Escaneo I2C rápido
  i2cScan(tw);

  Serial.println("Listo. Toque la pantalla para ver coordenadas.");

  pinMode(PIN_INT, INPUT_PULLUP);
  delay(50);
  Serial.println("Minidiagnostico GT911 - I2C productID y INT");
  // Leer ProductID 0x8140..0x8147
  uint16_t reg = 0x8140;
  tw.beginTransmission(I2C_ADDR);
  tw.write(reg >> 8);
  tw.write(reg & 0xFF);
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
unsigned long lastPrint=0;
uint32_t lastScan = 0;

void loop() {
  static uint16_t x=0, y=0;

  // Revisamos toque mediante LovyanGFX
  if (tft.getTouch(&x, &y)) {
    Serial.printf("Toque detectado X=%d Y=%d\n", x, y);

    // Indicador visual: dibujar un círculo y texto
    tft.fillCircle(x, y, 10, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextSize(1);
    tft.drawString(String(x) + "," + String(y), x + 16, y - 6);

    delay(200);
    // limpiar pequeño marcador
    tft.fillCircle(x, y, 10, TFT_BLACK);
  }

  // Cada 8 segundos, escanear I2C nuevamente y mostrar estado INT
  if (millis() - lastScan > 8000) {
    lastScan = millis();
    Serial.print("Estado pin INT: ");
    int state = digitalRead(PIN_INT);
    Serial.println(state == LOW ? "LOW (posible interrupcion)" : "HIGH");
    i2cScan(tw);
  }

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
