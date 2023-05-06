/*
    based on W. Hoogervorst, based on based on the 'CameraWebServer' example from the Arduino IDE for the AI Thinker ESP32-CAM board
    7.5 inch Waveshare e-paper B/W
    resolution 800x480
*/
#include "esp_camera.h"
//#include <WiFi.h>
#include <img_converters.h>

// include library, include base class, make path known
#include <GxEPD.h>
#include <GxGDEW075T7/GxGDEW075T7.h>      // 7.5" b/w 800x480

#include <GxGDEW075T7/BitmapExamples.h>

// FreeFonts from Adafruit_GFX
//#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/FreeSansBold12pt7b.h>
//#include <Fonts/FreeSansBold18pt7b.h>
//#include <Fonts/FreeSans9pt7b.h>
//#include <Fonts/FreeSans12pt7b.h>
//#include <Fonts/FreeSans18pt7b.h>

#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

const char* ssid = "FABIO ";
const char* password = "1qaz3edc";

// for SPI pin definitions see e.g.:
// C:\Users\xxx\Documents\Arduino\hardware\espressif\esp32\variants\lolin32\pins_arduino.h

//GxIO_Class io(SPI, /*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16); // arbitrary selection of 17, 16
//GxEPD_Class display(io, /*RST=*/ 16, /*BUSY=*/ 4); // arbitrary selection of (16), 4

SPIClass  epaper_SPI(HSPI); // define the SPI for the e-paper on the HSPI pins (CLK 14, MOSI 13, CS 15)
/*
  #define CS 15
  #define DC 4
  #define RST 2
  #define BUSY 16
*/
GxIO_Class io(epaper_SPI, /*CS*/ 15, /*DC*/ 4, /*RST*/ 2);
GxEPD_Class display(io, /*RST*/ 2,  /*BUSY*/ 16 );

// Select camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#define WIFI_CONNECT_TIMEOUT_S 15

// WiFiClient espClient;

boolean alarmstate;
boolean updatescreen = false;

void setupLedFlash(int pin);
void enable_led(bool en);
const GFXfont* f;

void setup() {
  Serial.begin(115200);
  delay(5000); // wait ArduinoIDE
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("init 000");
  Serial.println("[APP] memory");
  show_memory();

  // pinMode(14,INPUT_PULLUP);
  // pinMode(15,INPUT_PULLUP); 
  // Wire.begin(14,15);
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  digitalWrite(4, LOW);
  Serial.println("pin mode started");
  display.init(115200);
  display.setRotation(2);
  Serial.println("display initialized");

  // camera init
  // init_camera();

  // setup_wifi();

  //Serial.println("display printed");

  //Serial.print("Camera Ready! Use 'http://");
  //Serial.print(WiFi.localIP());
  //Serial.println("' to connect");
}

void loop() {
  Serial.println("[1] memory");
  show_memory();

  // uint8_t * buf = NULL;
  // size_t buf_len = 0;
  // enable_led(false);
  // if(take_photo(&buf, &buf_len) == ESP_FAIL) {
  //   Serial.println("Fail to take photo");
  //   return;
  // }
  
  // Serial.println("[1.1] memory");
  // show_memory();

  // Serial.print("buffer length:");Serial.println(buf_len);
  // esp_err_t err = esp_camera_deinit();
  // if(err != ESP_OK) {
  //   Serial.printf("Camera deinit failed with error 0x%x", err);
  // }

  // Serial.println("[2] memory");
  // show_memory();

  // if(buf_len > 0) {
    display.drawExampleBitmap(BitmapExample1, sizeof(BitmapExample1));
  //   //delay(5000);
  //   //display.drawBitmap(buf, sizeof(buf));
    // drawBitmap(buf, buf_len);
  //   Serial.println("image printed");

    // Serial.println("cleaning memory");
  //   Serial.println("[3] memory");
  //   show_memory();
    // free(buf);
    // buf_len = 0;
    //display.powerDown();
  // }
  Serial.println("[4] memory");
  show_memory();
  delay(10000);
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

// void setup_wifi() {
//   // We start by connecting to a WiFi network
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(ssid);
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);
//   WiFi.setSleep(false);

//   uint32_t time1, time2;
//   time1 = millis();
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//     time2 = millis();
//     if ((time2 - time1) > 1000 * WIFI_CONNECT_TIMEOUT_S) { // wifi connection lasts too long
//       break;
//       ESP.restart();
//     }
//     if (time1 > time2)
//       time1 = 0;
//   }
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
// }

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
  config.frame_size = FRAMESIZE_240X240;
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
      config.frame_size = FRAMESIZE_240X240;
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
  s->set_framesize(s, FRAMESIZE_240X240);
  enable_led(false);
}

static const uint16_t max_palette_pixels = 256; // for depth <= 8

// const char* host, const char* path, const char* filename, int16_t x, int16_t y, bool with_color
void drawBitmap2(uint8_t *bmp, size_t bmp_len) {
  int16_t x = (display.width() - 240) / 2;
  int16_t y = (display.height() - 240) / 2;
  if ((x >= display.width()) || (y >= display.height())) return;
  
  uint8_t mono_palette_buffer[max_palette_pixels / 8]; 
  //uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w

  Serial.println("[drawBitmap] memory");
  show_memory();

  uint32_t startTime = millis();
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  size_t start = 0;
 
  bool with_color = false;
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
          //colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
          if (0 == pn % 8) mono_palette_buffer[pn / 8] = 0;
          mono_palette_buffer[pn / 8] |= whitish << pn % 8;
          //!if (0 == pn % 8) color_palette_buffer[pn / 8] = 0;
          //!color_palette_buffer[pn / 8] |= colored << pn % 8;
          //Serial.print("0x00"); Serial.print(red, HEX); Serial.print(green, HEX); Serial.print(blue, HEX);
          //Serial.print(" : "); Serial.print(whitish); Serial.print(", "); Serial.println(colored);
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
                //!colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                in_byte <<= depth;
                in_bits -= depth;
              }
              break;
          }
          if (whitish) {
            color = GxEPD_WHITE;
          //!} else if (colored && with_color) {
          //!  color = GxEPD_RED;
          } else {
            color = GxEPD_BLACK;
          }
          uint16_t yrow = y + (flip ? h - row - 1 : row);
          display.drawPixel(x + col, yrow, color);
          
          if(color == GxEPD_BLACK) {
            Serial.print("X");
          } else {
            Serial.print(" ");
          }  
          //Serial.print(x + col);Serial.print("x");Serial.println(yrow);
        } // end pixel
        Serial.println();
      } // end line
    }
    Serial.print("bytes read (start): "); Serial.println(start);
    Serial.print("loaded in "); Serial.print(millis() - startTime); Serial.println(" ms");
    display.update();
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

void show_memory() {
  Serial.print("Total heap: ");Serial.println(ESP.getHeapSize());
  Serial.print("Free heap: ");Serial.println(ESP.getFreeHeap());
  Serial.print("Total PSRAM: ");Serial.println(ESP.getPsramSize());
  Serial.print("Free PSRAM: ");Serial.println(ESP.getFreePsram());
}
