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

#define LED_LEDC_CHANNEL 2 // Using different ledc channel/timer than camera
#define CONFIG_LED_MAX_INTENSITY 255

// #include <GxEPD2_BW.h>
#include <GxEPD2_4G_4G.h>
#include <GxEPD2_4G_BW.h>
#include <esp_camera.h>
#include <img_converters.h>

/* Conexao do epaper:
BUSY - ROXO    - IO12
RST  - BRANCO  - IO2
DC   - VERDE   - IO3/UnR (pode usar o IO4 durante o desenvolvimento)
CS   - LARANJA - IO15
CLK  - AMARELO - IO14
DIN  - AZUL    - IO13
GND  - MARROM  - GND (proximo ao 3v3)
VCC  - CINZA   - 3V3
*/
#define GxEPD2_DRIVER_CLASS GxEPD2_750_T7  // GDEW075T7   800x480, GD7965
#define MAX_DISPLAY_BUFFER_SIZE 65536ul // e.g.
#define MAX_HEIGHT_BW(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))
#define MAX_HEIGHT_4G(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 4) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 4))
// adapt the constructor parameters to your wiring
GxEPD2_4G_4G<GxEPD2_DRIVER_CLASS, MAX_HEIGHT_4G(GxEPD2_DRIVER_CLASS)> display(GxEPD2_750_T7(/*CS=5*/ 15, /*DC=*/ 3, /*RST=*/ 2, /*BUSY=*/ 12));
GxEPD2_4G_BW_R<GxEPD2_DRIVER_CLASS, MAX_HEIGHT_BW(GxEPD2_DRIVER_CLASS)> display_bw(display.epd2);
#undef MAX_DISPLAY_BUFFER_SIZE
#undef MAX_HEIGHT_BW
#undef MAX_HEIGHT_4G
#undef GxEPD2_DRIVER_CLASS

//#define SHOW_GRAY_SCALE // Teste screen gray scale

SPIClass hspi(HSPI);

void setup() {
  Serial.begin(115200);
  delay(5000); // wait ArduinoIDE
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("setup");

  show_memory("[setup] memory");

  init_camera();
  delay(5000); // wait ArduinoIDE

  hspi.begin(14, 12, 13, 15); // remap hspi for EPD (swap pins)(sck, miso, mosi, ss)
  display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  Serial.println("hspi configured");

  display.init(115200, true, 2, false);
  Serial.println();
  Serial.println("display initialized");

#ifdef SHOW_GRAY_SCALE
  showGreyLevels();
#endif
}

void loop() {
  Serial.println("loop");
  show_memory("[l1] memory");

  uint8_t * buf = NULL;
  size_t buf_len = 0;
  enable_led(false);
  if(take_photo(&buf, &buf_len) == ESP_FAIL) {
    Serial.println("Fail to take photo");
  }
  enable_led(false);
  
  // Serial.print("buffer length:");Serial.println(buf_len);
  // esp_err_t err = esp_camera_deinit();
  // if(err != ESP_OK) {
  //   Serial.printf("Camera deinit failed with error 0x%x", err);
  // }

  if(buf_len > 0) {
    display.setRotation(0);
    drawBitmap(buf, buf_len, true);
    Serial.println("image printed");

    Serial.println("cleaning memory");
    show_memory("[3] memory");
    free(buf);
    buf_len = 0;
  }

  show_memory("[l2] memory");
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
  config.jpeg_quality = 10;
  config.fb_count = 1;
  
  if(config.pixel_format == PIXFORMAT_JPEG){
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else if(config.pixel_format != PIXFORMAT_GRAYSCALE) {
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
  */

  // Setup LED FLash if LED pin is defined in camera_pins.h
  setupLedFlash(LED_GPIO_NUM);

  //s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_special_effect(s, 2);
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
    Serial.print("Set LED intensity to ");Serial.println(duty);
}

static const uint16_t input_buffer_pixels = 800; // may affect performance

static const uint16_t max_row_width = 1872; // for up to 7.8" display 1872x1404
static const uint16_t max_palette_pixels = 256; // for depth <= 8

uint8_t input_buffer[3 * input_buffer_pixels]; // up to depth 24
uint8_t output_row_mono_buffer[max_row_width / 8]; // buffer for at least one row of b/w bits
uint8_t output_row_color_buffer[max_row_width / 8]; // buffer for at least one row of color bits
uint8_t mono_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 b/w
uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w
uint16_t rgb_palette_buffer[max_palette_pixels]; // palette buffer for depth <= 8 for buffered graphics, needed for 7-color display

//void drawBitmapFromSpiffs_Buffered(const char *filename, int16_t x, int16_t y, bool with_color, bool partial_update, bool overwrite)
void drawBitmap(uint8_t *bmp, size_t bmp_len, bool with_color) {
  size_t start = 0;
  bool partial_update = false; 
  bool overwrite = false;
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  bool has_multicolors = true; // (display.epd2.panel == GxEPD2::ACeP565) || (display.epd2.panel == GxEPD2::GDEY073D46);
  uint32_t startTime = millis();
  //if ((x >= display.width()) || (y >= display.height())) return;
  // Parse BMP header
  if (read16(bmp, bmp_len, start) == 0x4D42) { // BMP signature
    uint32_t fileSize = read32(bmp, bmp_len, start);
    uint32_t creatorBytes = read32(bmp, bmp_len, start); (void)creatorBytes; //unused
    uint32_t imageOffset = read32(bmp, bmp_len, start); // Start of image data
    uint32_t headerSize = read32(bmp, bmp_len, start);
    uint32_t width  = read32(bmp, bmp_len, start);
    int32_t height = (int32_t) read32(bmp, bmp_len, start);
    uint16_t planes = read16(bmp, bmp_len, start);
    uint16_t depth = read16(bmp, bmp_len, start); // bits per pixel
    uint32_t format = read32(bmp, bmp_len, start);

    if ((planes == 1) && ((format == 0) || (format == 3))) { // uncompressed is handled, 565 also
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

      int16_t x = (display.width() - width) / 2;
      int16_t y = (display.height() - height) / 2;

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
        start = imageOffset; //palette is always @ 54
        for (uint16_t pn = 0; pn < (1 << depth); pn++) {
          blue  = read8(bmp, bmp_len, start);
          green = read8(bmp, bmp_len, start);
          red   = read8(bmp, bmp_len, start);
          start++;
          whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
          colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
          if (0 == pn % 8) mono_palette_buffer[pn / 8] = 0;
          mono_palette_buffer[pn / 8] |= whitish << pn % 8;
          if (0 == pn % 8) color_palette_buffer[pn / 8] = 0;
          color_palette_buffer[pn / 8] |= colored << pn % 8;
          rgb_palette_buffer[pn] = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
        }
      }
      if (partial_update) display.setPartialWindow(x, y, w, h);
      else display.setFullWindow();
      display.firstPage();
      do {
        if (!overwrite) display.fillScreen(GxEPD_WHITE);
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) {
          uint8_t in_byte = 0; // for depth <= 8
          uint8_t in_bits = 0; // for depth <= 8
          uint16_t color = GxEPD_WHITE;
          start = rowPosition;
          for (uint16_t col = 0; col < w; col++) { // for each pixel
            switch (depth) {
              case 32:
                blue = read8(bmp, bmp_len, start);
                green = read8(bmp, bmp_len, start);
                red = read8(bmp, bmp_len, start);
                start++; // skip alpha
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
                break;
              case 24:
                blue = read8(bmp, bmp_len, start);
                green = read8(bmp, bmp_len, start);
                red = read8(bmp, bmp_len, start);
                whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
                break;
              case 16:
                {
                  uint8_t lsb = read8(bmp, bmp_len, start);
                  uint8_t msb = read8(bmp, bmp_len, start);
                  if (format == 0) // 555
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                    red   = (msb & 0x7C) << 1;
                    color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
                  }
                  else // 565
                  {
                    blue  = (lsb & 0x1F) << 3;
                    green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                    red   = (msb & 0xF8);
                    color = (msb << 8) | lsb;
                  }
                  whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
                  colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0)); // reddish or yellowish?
                }
                break;
              case 1:
              case 2:
              case 4:
              case 8:
                {
                  if (0 == in_bits)
                  {
                    in_byte = read8(bmp, bmp_len, start);
                    in_bits = 8;
                  }
                  uint16_t pn = (in_byte >> bitshift) & bitmask;
                  whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
                  in_byte <<= depth;
                  in_bits -= depth;
                  color = rgb_palette_buffer[pn];
                }
                break;
            }
            if (with_color && has_multicolors) {
              // keep color
            } else if (whitish) {
              color = GxEPD_WHITE;
            } else if (colored && with_color) {
              color = GxEPD_COLORED;
            } else {
              color = GxEPD_BLACK;
            }
            uint16_t yrow = y + (flip ? h - row - 1 : row);
            display.drawPixel(x + col, yrow, color);
          } // end pixel
        } // end line
      }
      while (display.nextPage());
      Serial.print("loaded in "); Serial.print(millis() - startTime); Serial.println(" ms");
    }
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

#ifdef SHOW_GRAY_SCALE
void showGreyLevels() {
  Serial.println("Gray levels example:");
  display.clearScreen();
  display.setRotation(0);
  uint16_t h = display.height() / 4;
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.fillRect(0, 0, display.width(), h, GxEPD_WHITE);
    display.fillRect(0, h, display.width(), h, GxEPD_LIGHTGREY);
    display.fillRect(0, 2 * h, display.width(), h, GxEPD_DARKGREY);
    display.fillRect(0, 3 * h, display.width(), h, GxEPD_BLACK);
  }
  while (display.nextPage());
  Serial.println("Gray levels printed.");
  delay(2000);
}
#endif

/*
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
} Pixel;

int main() {
    FILE *input_file = fopen("input.bmp", "rb");
    FILE *output_file = fopen("output.bmp", "wb");

    // Read BMP header
    // ...

    // Read pixel data
    Pixel *pixels = malloc(sizeof(Pixel) * width * height);
    fread(pixels, sizeof(Pixel), width * height, input_file);

    // Convert pixels to grayscale
    for (int i = 0; i < width * height; i++) {
        unsigned char gray = 0.3 * pixels[i].red + 0.59 * pixels[i].green + 0.11 * pixels[i].blue;
        unsigned char level = gray / 64;
        pixels[i].red = level * 64;
        pixels[i].green = level * 64;
        pixels[i].blue = level * 64;
    }

    // Write new pixel data
    fwrite(pixels, sizeof(Pixel), width * height, output_file);

    fclose(input_file);
    fclose(output_file);
}



#include <GxEPD2.h>
#include <GxGDEW042T2/GxGDEW042T2.h>

#define EPD_CS 5
#define EPD_DC 17
#define EPD_RST 16
#define EPD_BUSY 4

GxEPD2_BW<GxEPD2_420_T2> display(GxEPD2_420_T2(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

typedef struct {
    unsigned char blue;
    unsigned char green;
    unsigned char red;
} Pixel;

void setup() {
    display.init(115200);
}

void loop() {
    FILE *input_file = fopen("input.bmp", "rb");

    // Read BMP header
    // ...

    // Read pixel data
    Pixel *pixels = malloc(sizeof(Pixel) * width * height);
    fread(pixels, sizeof(Pixel), width * height, input_file);

    // Convert pixels to grayscale
    for (int i = 0; i < width * height; i++) {
        unsigned char gray = 0.3 * pixels[i].red + 0.59 * pixels[i].green + 0.11 * pixels[i].blue;
        unsigned char level = gray / 64;
        pixels[i].red = level * 64;
        pixels[i].green = level * 64;
        pixels[i].blue = level * 64;
    }

    // Write new pixel data to display
    display.fillScreen(GxEPD_WHITE);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int i = y * width + x;
            if (pixels[i].red == 0) {
                display.drawPixel(x, y, GxEPD_BLACK);
            } else if (pixels[i].red == 64) {
                display.drawPixel(x, y, GxEPD_DARK_GRAY);
            } else if (pixels[i].red == 128) {
                display.drawPixel(x, y, GxEPD_LIGHT_GRAY);
            } else if (pixels[i].red == 192) {
                display.drawPixel(x, y, GxEPD_WHITE);
            }
        }
    }
    display.update();

    fclose(input_file);
}
*/