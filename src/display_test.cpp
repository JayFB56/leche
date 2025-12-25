// Integración LVGL + LovyanGFX + GT911 (teclado numérico)

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <lvgl.h>

#define PIN_SDA 33
#define PIN_SCL 32
#define PIN_INT 18
#define I2C_ADDR 0x5D

class LGFX_ESP32_3248S035C : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
public:
  LGFX_ESP32_3248S035C() {
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

    auto pcfg = _panel_instance.config();
    pcfg.pin_cs = 15;
    pcfg.pin_rst = -1;
    pcfg.panel_width = 320;
    pcfg.panel_height = 480;
    pcfg.bus_shared = true;
    pcfg.offset_rotation = 0;
    _panel_instance.config(pcfg);

    setPanel(&_panel_instance);

    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);
  }
};

LGFX_ESP32_3248S035C tft;
TwoWire tw = TwoWire(1);

// GT911 I2C helpers (usados para leer touch)
bool gt911_read(uint16_t reg, uint8_t *buf, size_t len) {
  tw.beginTransmission(I2C_ADDR);
  tw.write((uint8_t)(reg >> 8));
  tw.write((uint8_t)(reg & 0xFF));
  if (tw.endTransmission(false) != 0) return false;
  size_t r = tw.requestFrom((int)I2C_ADDR, (uint8_t)len);
  if (r != len) return false;
  for (size_t i=0;i<len;i++) buf[i] = tw.read();
  return true;
}

bool gt911_write(uint16_t reg, const uint8_t *buf, size_t len) {
  tw.beginTransmission(I2C_ADDR);
  tw.write((uint8_t)(reg >> 8));
  tw.write((uint8_t)(reg & 0xFF));
  for (size_t i=0;i<len;i++) tw.write(buf[i]);
  return (tw.endTransmission() == 0);
}

// Variables para intercambio entre el polling del touch y LVGL
static volatile int16_t lv_touch_x = -1;
static volatile int16_t lv_touch_y = -1;
static volatile bool lv_touch_pressed = false;

// Buffer para LVGL (alto por partes)
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;

// Forward
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void my_indev_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Inicializando LovyanGFX + LVGL...");

  tft.init();
  tft.setRotation(5);
  tft.fillScreen(TFT_BLACK);

  // I2C para GT911
  tw.begin(PIN_SDA, PIN_SCL, 400000);
  pinMode(PIN_INT, INPUT_PULLUP);
  delay(50);

  // Inicializar LVGL
  lv_init();

  int scr_w = tft.width();
  int buf_h = 80; // filas de buffer
  buf1 = (lv_color_t*)malloc(sizeof(lv_color_t) * scr_w * buf_h);
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, scr_w * buf_h);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.hor_res = scr_w;
  disp_drv.ver_res = tft.height();
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_indev_read;
  lv_indev_drv_register(&indev_drv);

  // Crear UI: textarea y teclado numérico (btnmatrix)
  lv_obj_t * ta = lv_textarea_create(lv_scr_act());
  lv_obj_set_size(ta, tft.width()-20, 60);
  lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 10);
  lv_textarea_set_text(ta, "");
  lv_textarea_set_placeholder_text(ta, "Ingrese números...");

  static const char * btnm_map[] = {
    "1","2","3","\n",
    "4","5","6","\n",
    "7","8","9","\n",
    "<-","0","OK", ""
  };

  lv_obj_t * btnm = lv_btnmatrix_create(lv_scr_act());
  lv_btnmatrix_set_map(btnm, btnm_map);
  lv_obj_set_width(btnm, tft.width()-20);
  lv_obj_align(btnm, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_event_cb(btnm, [](lv_event_t * e){
    lv_obj_t * ta = lv_scr_act();
    // buscar el textarea creado previamente (primer hijo)
    lv_obj_t * child = lv_obj_get_child(lv_scr_act(), 0);
    if (!child) return;
    lv_obj_t * ta_obj = child; // asumimos es la textarea
    lv_obj_t * btnm = lv_event_get_target(e);
    int id = lv_btnmatrix_get_active_btn(btnm);
    if (id < 0) return;
    const char * txt = lv_btnmatrix_get_btn_text(btnm, id);
    if (!txt) return;
    if (strcmp(txt, "<-") == 0) {
      lv_textarea_del_char(ta_obj);
    } else if (strcmp(txt, "OK") == 0) {
      // opcional: acción OK
    } else {
      lv_textarea_add_text(ta_obj, txt);
    }
  }, LV_EVENT_VALUE_CHANGED, NULL);

  Serial.println("Inicialización completada.");
}

// Display flush: envia los pixeles al tft
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;
  // LGFX/TFT espera 16-bit colores (RGB565). lv_color_t suele ser 16-bit con LV_COLOR_DEPTH=16
  tft.pushImage(area->x1, area->y1, w, h, (uint16_t*)color_p);
  lv_disp_flush_ready(disp);
}

// Indev read: LVGL preguntará por el estado; usamos variables actualizadas por el loop
static void my_indev_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  if (lv_touch_pressed) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = lv_touch_x;
    data->point.y = lv_touch_y;
  } else {
    data->state = LV_INDEV_STATE_REL;
    data->point.x = lv_touch_x;
    data->point.y = lv_touch_y;
  }
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 5) {
    last = now;
    lv_timer_handler();
  }

  // Poll GT911 y actualizar variables
  uint8_t status = 0;
  if (gt911_read(0x814E, &status, 1)) {
    uint8_t touches = status & 0x0F;
    if (touches > 0 && touches <= 5) {
      uint8_t buf[8*5];
      if (gt911_read(0x8150, buf, 8*touches)) {
        int idx = 0; // primer punto
        uint16_t x = (uint16_t)buf[idx+1] | ((uint16_t)buf[idx+2] << 8);
        uint16_t y = (uint16_t)buf[idx+3] | ((uint16_t)buf[idx+4] << 8);
        lv_touch_x = x;
        lv_touch_y = y;
        lv_touch_pressed = true;
        // limpiar estado
        uint8_t zero = 0;
        gt911_write(0x814E, &zero, 1);
      }
    } else {
      lv_touch_pressed = false;
    }
  }
}
