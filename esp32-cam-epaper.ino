#define ENABLE_GxEPD2_GFX 0

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

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM       4

#define LED_LEDC_CHANNEL 2 //Using different ledc channel/timer than camera
#define CONFIG_LED_MAX_INTENSITY 255

#include <GxEPD2_BW.h>
#include "bitmaps/Bitmaps800x480.h" // 7.5"  b/w

#include "esp_camera.h"
#include <img_converters.h>

/* Conexao do epaper:
BUSY - ROXO    - IO12
RST  - BRANCO  - IO2
DC   - VERDE   - IO4
CS   - LARANJA - IO15
CLK  - AMARELO - IO14
DIN  - AZUL    - IO13
GND  - MARROM  - GND (proximo ao 3v3)
VCC  - CINZA   - 3V3
*/
// hspi.begin(14, 12, 13, 15); // remap hspi for EPD (swap pins)(sck, miso, mosi, ss)
//GxEPD2_BW<GxEPD2_750_T7, GxEPD2_750_T7::HEIGHT> display(GxEPD2_750_T7(/*CS=*/ 15, /*DC=*/ 4, /*RST=*/ 2, /*BUSY=*/ 16)); // GDEW075T7 800x480, EK79655 (GD7965)
GxEPD2_BW<GxEPD2_750_YT7, GxEPD2_750_YT7::HEIGHT> display(GxEPD2_750_YT7(/*CS=*/ 15, /*DC=*/ 4, /*RST=*/ 2, /*BUSY=*/ 12)); // GDEY075T7 800x480, UC8179 (GD7965)
SPIClass hspi(HSPI);

void setup() {
  Serial.begin(115200);
  delay(5000); // wait ArduinoIDE
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("setup");

  show_memory("[1] memory");

  init_camera();

  hspi.begin(14, 12, 13, 15); // remap hspi for EPD (swap pins)(sck, miso, mosi, ss)
  display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  Serial.println("hspi configured");

  display.init(115200, true, 2, false);
  Serial.println();
  Serial.println("display initialized");

  ////////////////////////////
  uint8_t * buf = NULL;
  size_t buf_len = 0;
  enable_led(false);
  if(take_photo(&buf, &buf_len) == ESP_FAIL) {
    Serial.println("Fail to take photo");
    return;
  }
  
  Serial.print("buffer length:");Serial.println(buf_len);
  esp_err_t err = esp_camera_deinit();
  if(err != ESP_OK) {
    Serial.printf("Camera deinit failed with error 0x%x", err);
  }

  if(buf_len > 0) {
    display.setRotation(0);
    drawBitmap(buf, buf_len);
    Serial.println("image printed");

    Serial.println("cleaning memory");
    show_memory("[3] memory");
    free(buf);
    buf_len = 0;
  }
  
  show_memory("[2] memory");
}

void loop() {
  Serial.println("loop");
  delay(10000);
}

void init_camera(void) {
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
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG; // PIXFORMAT_JPEG | PIXFORMAT_GRAYSCALE; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  /*
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }
  */

  // Setup LED FLash if LED pin is defined in camera_pins.h
  setupLedFlash(LED_GPIO_NUM);

  //s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_framesize(s, FRAMESIZE_VGA);
  enable_led(false);
}

esp_err_t take_photo(uint8_t ** out, size_t * out_len) {
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
  }

  if (fb->format == PIXFORMAT_GRAYSCALE || fb->format == PIXFORMAT_JPEG) {
    Serial.println("format: PIXFORMAT_JPEG || PIXFORMAT_GRAYSCALE");
    if(frame2bmp(fb, out, out_len)) {
      Serial.println("conversion frame2bmp done");
    }
    esp_camera_fb_return(fb);
  } else {
    Serial.println("format: UNKNOWN");
    esp_camera_fb_return(fb);
    return ESP_FAIL;
  }

  return 0;
}

void setupLedFlash(int pin) 
{
    ledcSetup(LED_LEDC_CHANNEL, 5000, 8);
    ledcAttachPin(pin, LED_LEDC_CHANNEL);
}

void enable_led(bool en) { // Turn LED On or Off
    int duty = en ? CONFIG_LED_MAX_INTENSITY : 0;
    ledcWrite(LED_LEDC_CHANNEL, duty);
    //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
    Serial.print("Set LED intensity to ");Serial.println(duty);
}

static const uint16_t max_palette_pixels = 256; // for depth <= 8

// const char* host, const char* path, const char* filename, int16_t x, int16_t y, bool with_color
void drawBitmap(uint8_t *bmp, size_t bmp_len) {
  int16_t x = (display.width() - 640) / 2;
  int16_t y = (display.height() - 480) / 2;
  if ((x >= display.width()) || (y >= display.height())) return;
  
  uint8_t mono_palette_buffer[max_palette_pixels / 8]; 
  uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w

  uint32_t startTime = millis();
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  size_t start = 0;
 
  bool with_color = true;
  display.firstPage();
  display.fillScreen(GxEPD_WHITE);

  // Parse BMP header
  if (read16(bmp, bmp_len, start) == 0x4D42) { // BMP signature
    Serial.println("is a bitmap");
    int32_t fileSize = read32(bmp, bmp_len, start);
    int32_t creatorBytes = read32(bmp, bmp_len, start);
    int32_t imageOffset = read32(bmp, bmp_len, start); // Start of image data
    int32_t headerSize = read32(bmp, bmp_len, start);
    int32_t width  = read32(bmp, bmp_len, start);
    int32_t height = read32(bmp, bmp_len, start);
    uint16_t planes = read16(bmp, bmp_len, start);
    uint16_t depth = read16(bmp, bmp_len, start); // bits per pixel
    int32_t format = read32(bmp, bmp_len, start);
    if ((planes == 1) && ((format == 0) || (format == 3))) {// uncompressed is handled, 565 also
      Serial.print("start: "); Serial.println(start);
      Serial.print("File size: "); Serial.println(fileSize);
      Serial.print("Image Offset: "); Serial.println(imageOffset);
      Serial.print("Header size: "); Serial.println(headerSize);
      Serial.print("Bit Depth: "); Serial.println(depth);
      Serial.print("Image size: ");Serial.print(width);Serial.print('x');Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (depth < 8) rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;
      if (height < 0) {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      if ((x + w - 1) >= display.width())  w = display.width()  - x;
      if ((y + h - 1) >= display.height()) h = display.height() - y;
      valid = true;
      uint8_t bitmask = 0xFF;
      uint8_t bitshift = 8 - depth;
      uint16_t red, green, blue;
      bool whitish = false;
      bool colored = false;
      if (depth == 1) with_color = false;
      if (depth <= 8) {
        if (depth < 8) bitmask >>= depth;
        start = 54; // 54 for regular, diff for colorsimportant
        for (uint16_t pn = 0; pn < (1 << depth); pn++) {
          blue  = read8(bmp, bmp_len, start);
          green = read8(bmp, bmp_len, start);
          red   = read8(bmp, bmp_len, start);
          read8(bmp, bmp_len, start);
          whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
          colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
          if (0 == pn % 8) mono_palette_buffer[pn / 8] = 0;
          mono_palette_buffer[pn / 8] |= whitish << pn % 8;
          if (0 == pn % 8) color_palette_buffer[pn / 8] = 0;
          color_palette_buffer[pn / 8] |= colored << pn % 8;
        }
      }
      display.fillScreen(GxEPD_WHITE);
      start = flip ? imageOffset + (height - h) * rowSize : imageOffset;
      Serial.println("Printing image ...");
      for (uint16_t row = 0; row < h; row++) { // for each line
        uint8_t in_byte = 0; // for depth <= 8
        uint8_t in_bits = 0; // for depth <= 8
        uint16_t color = GxEPD_WHITE;
        for (uint16_t col = 0; col < w; col++) { // for each pixel
          // Time to read more pixel data?
          switch (depth) {
            case 24:
              blue = read8(bmp, bmp_len, start);
              green = read8(bmp, bmp_len, start);
              red = read8(bmp, bmp_len, start);
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
              break;
            case 16:{
                uint8_t lsb = read8(bmp, bmp_len, start);
                uint8_t msb = read8(bmp, bmp_len, start);
                if (format == 0) { // 555
                  blue  = (lsb & 0x1F) << 3;
                  green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                  red   = (msb & 0x7C) << 1;
                } else { // 565
                  blue  = (lsb & 0x1F) << 3;
                  green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                  red   = (msb & 0xF8);
                }
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
              }
              break;
            case 1:
            case 4:
            case 8:{
                if (0 == in_bits) {
                  in_byte = read8(bmp, bmp_len, start);
                  in_bits = 8;
                }
                uint16_t pn = (in_byte >> bitshift) & bitmask;
                whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
                colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                in_byte <<= depth;
                in_bits -= depth;
              }
              break;
          }
          if (whitish) {
            color = GxEPD_WHITE;
          } else if (colored && with_color) {
            color = GxEPD_COLORED;
          } else {
            color = GxEPD_BLACK;
          }
          uint16_t yrow = y + (flip ? h - row - 1 : row);
          display.drawPixel(x + col, yrow, color);
          
          // if(color == GxEPD_BLACK) {
          //   Serial.print("X");
          // } else {
          //   Serial.print(" ");
          // }  
          //Serial.print(x + col);Serial.print("x");Serial.println(yrow);
        } // end pixel
        //Serial.println();
      } // end line
    }
    Serial.print("bytes read (start): "); Serial.println(start);
    Serial.print("loaded in "); Serial.print(millis() - startTime); Serial.println(" ms");
    while(display.nextPage());
    // display.update();
  }  
  if (!valid) {
    Serial.println("bitmap format not handled.");
  }
}

uint8_t read8(uint8_t buf[], size_t buf_len, size_t &start) {
  if(start + 1 > buf_len) {
    Serial.println("Skipping out of index buffer read.");
     return 0;
  }
  return buf[start++];
}

uint16_t read16(uint8_t buf[], size_t buf_len, size_t &start) {
  if(start + 2 > buf_len) {
    Serial.println("Skipping out of index buffer read.");
     return 0;
  }
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = buf[start++]; // LSB
  ((uint8_t *)&result)[1] = buf[start++]; // MSB
  return result;
}

int32_t read32(uint8_t buf[], size_t buf_len, size_t &start) {
  if(start + 4 > buf_len) {
    Serial.println("Skipping out of index buffer read.");
     return 0;
  }
  // BMP data is stored little-endian, same as Arduino.
  int32_t result;
  ((uint8_t *)&result)[0] = buf[start++]; // LSB
  ((uint8_t *)&result)[1] = buf[start++];
  ((uint8_t *)&result)[2] = buf[start++];
  ((uint8_t *)&result)[3] = buf[start++]; // MSB
  return result;
}

void show_memory(char arr[]) {
  Serial.println(arr);
  Serial.print("Total heap: ");Serial.println(ESP.getHeapSize());
  Serial.print("Free heap: ");Serial.println(ESP.getFreeHeap());
  Serial.print("Total PSRAM: ");Serial.println(ESP.getPsramSize());
  Serial.print("Free PSRAM: ");Serial.println(ESP.getFreePsram());
}
