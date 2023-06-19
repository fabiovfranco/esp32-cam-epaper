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


typedef struct {
  uint32_t filesize;
  uint32_t reserved;
  uint32_t offset;
  uint32_t headersize;
  int32_t width;
  int32_t height;
  uint16_t planes;
  uint16_t bitsPerPixel; // depth
  uint32_t compression; // format
  uint32_t imageSize;
  uint32_t xPixelsPerMeter;
  uint32_t yPixelsPerMeter;
  uint32_t colorsUsed;
  uint32_t colorsImportant;
} bmp_header;

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

// #define SHOW_GRAY_SCALE // To check screen

SPIClass hspi(HSPI);

uint16_t gamme_curve[256];

void setup() {
  Serial.begin(115200);
  delay(1000); // wait ArduinoIDE
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("setup");
  show_memory("[setup] memory");

  init_camera();

  hspi.begin(14, 12, 13, 15); // remap hspi for EPD (swap pins)(sck, miso, mosi, ss)
  display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  Serial.println("hspi configured");

  display.init(115200, true, 2, false);
  Serial.println();
  Serial.println("display initialized");

  make_gamma_curve(0.8);
#ifdef SHOW_GRAY_SCALE
  showGreyLevels();
#endif
}

void loop() {
  Serial.println("loop");
  show_memory("[l1] memory");

  uint8_t * buf = NULL;
  size_t buf_len = 0;
  if(take_photo(&buf, &buf_len) == ESP_FAIL) {
    Serial.println("Fail to take photo");
  }
  
  if(buf_len > 0) {
    display.setRotation(0);
    //drawBitmap(buf, buf_len, false);
    //delay(2000);
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

void make_gamma_curve(double gamma_value) {
  // Affects the gamma to calculate gray (lower is darker/higher contrast)
  // Nice test values: 0.9 1.2 1.4 higher and is too bright
  double gammaCorrection = 1.0 / gamma_value;
  uint8_t val = 0;
  uint16_t color = 0;
  Serial.println("Gamma correction scale: ");
  for (int gray_value=0; gray_value<256;gray_value++) {
    val = round(255*pow(gray_value/255.0, gammaCorrection));
    //0, 85, 170, 255
    if (val <= 64) { // 64
      color = GxEPD_BLACK;
      // Serial.print(val);Serial.println(" B");
    } else if (val <= 128) { // 128
      color = GxEPD_DARKGREY;
      // Serial.print(val);Serial.println(" D");
    } else if (val <= 192) { // 192
      color = GxEPD_LIGHTGREY;
      // Serial.print(val);Serial.println(" L");
    } else {
      color = GxEPD_WHITE;
      // Serial.print(val);Serial.println(" W");
    }

    gamme_curve[gray_value] = color;
  }
  // Serial.println("");
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

  // Setup LED FLash if LED pin is defined in camera_pins.h
  setupLedFlash(LED_GPIO_NUM);

  //s->set_brightness(s, 1); // up the brightness just a bit
  //s->set_contrast(s, 2); // lower the saturation
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_special_effect(s, 2);
  enable_led(false);
}

esp_err_t take_photo(uint8_t ** out, size_t * out_len) {
  camera_fb_t *fb = NULL;
  delay(5000);
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

#define END (-32)

void dithering(uint8_t *bmp, uint16_t w, uint16_t h, uint8_t bpp) {
  Serial.printf("Start dithering (%dx%d) %dbpp\n", w, h ,bpp);
  uint8_t quantization_bits = 7;
  uint8_t weights[16];
  uint8_t len = 1;
  uint8_t max_filt_height = 0, max_filt_width = 0;
  int8_t _filters[] = {16, 7, -1, 3, 5, 1, END};
  for(; _filters[len] > END; len++) {
    weights[len] = _filters[len];
    if(weights[len] > 0) {
      if(max_filt_height == 0) {
        max_filt_width++;	 // find the filter right-side length; assuming the longest row is the first one.
      }
    } else {
      max_filt_height++;	// find the filter height
    }
  }

  uint8_t r = 0, newr = 0, dummy = 0, curr_weight = 0;
  int16_t temp_r = 0, quant_err_r = 0;

  bool bitshift_avail = 0;
  uint8_t bitshift_div = 0;
  uint8_t divisor = 16;
  if(is_2s_pow(divisor)) {
    bitshift_avail = 1;
    bitshift_div = twos_power(divisor);
  }
  
  for (uint16_t row = 0; row < h; row++) {
    for (uint16_t col = 0; col < w; col++) {
      uint32_t ind = index(col, row * w, bpp);
      uint8_t color = bmp[ind];
      colorGray256To888(color, r, dummy, dummy);
      newr = r;
      quantize(quantization_bits, newr, dummy, dummy);
      quant_err_r = r - newr;
      bmp[ind] = color888ToGray256(newr, newr, newr);

      // Distribute error amongst neighbours
      int8_t row_offs = 0, col_offs = 1;

      for(uint8_t p = 1; p < len; p++) {	//  fetch current filter weight
        // adjacent pixels work starts here. First, check if away from edge cases:
        if(row < (h - max_filt_height)  &&  col < (w - max_filt_width)  &&  col > 0) {
          
          // Fetch current dithering weight
          curr_weight = weights[p];
          
          if(curr_weight < 0) {		// Negative values in the weights array indicates to go down to the next line, and by which amount to the left
            col_offs = curr_weight;
            row_offs++;			// linefeed
          } else {		// If not outside image boundaries
            ind = index(col + col_offs, (row + row_offs) * w, bpp);
            col_offs++;
            
            // Fetch pixel and convert it to RGB888
            colorGray256To888(bmp[ind], r, dummy, dummy);
            
            if(!bitshift_avail) {		// If divisor is not a power of 2, bitshift division is not possible.
              temp_r = r + ((quant_err_r * curr_weight) / divisor);
            } else{				// On most uCs, bitshift division is quite cycle-expensive. Whenever possible, use bitshift. On the other hand, multiplication is often single-cycle.
              temp_r = r + ((quant_err_r * curr_weight) >> bitshift_div);
            }
            
            // Clamp pixel value if outside range [0-255]
            r = clamp(temp_r, 0, 255);
            
            // Assign new pixel value (each neighbour)
            bmp[ind] = color888ToGray256(r, r, r);
          }
        }
      }
    }
  }
  Serial.println("End dithering");
}

void drawBitmap(uint8_t *bmp, size_t bmp_len, bool dither) {
  if (bmp_len < 54 || bmp[0] != 'B' || bmp[1] != 'M') { // BMP signature
    Serial.print("Invalid bmp size.");
    return;
  }

  Serial.print("Image magic: ");Serial.print((char)bmp[0]);Serial.println((char)bmp[1]);
  if (bmp[0] != 'B' || bmp[1] != 'M') { // BMP signature
    Serial.print("Invalid bmp file signature.");
    return;
  }
  bmp_header *image = (bmp_header *)&bmp[2];
  uint8_t *data = &bmp[image->offset];
  
  bool valid = false; // valid format to be handled
  bool flip = true; // bitmap is stored bottom-to-top
  uint32_t startTime = millis();
  // Parse BMP header
  Serial.print("FileSize: "); Serial.println(image->filesize);
  Serial.print("Offset: "); Serial.println(image->offset);
  Serial.print("Bit Depth (bpp): "); Serial.println(image->bitsPerPixel);
  Serial.print("Compression: "); Serial.println(image->compression);
  if (image->bitsPerPixel == 24 && image->planes == 1 && ((image->compression == 0) || (image->compression == 3))) { // uncompressed is handled, 565 also
    Serial.print("Image size: "); Serial.println(image->imageSize);
    Serial.print("Image size: ");Serial.print(image->width);Serial.print('x');Serial.println(image->height);
    // BMP rows are padded (if needed) to 4-byte boundary
    uint32_t width = image->width;
    int32_t height = image->height;
    uint8_t bpp = image->bitsPerPixel / 8;
    uint32_t rowSize = (width * bpp + 3) & ~3;
    if (image->bitsPerPixel < 8) {
      rowSize = ((width * image->bitsPerPixel + 8 - image->bitsPerPixel) / 8 + 3) & ~3;
    }
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
    uint16_t red, green, blue;

    if (dither) {
      dithering(data, width, height, bpp);
    }
    
    display.setFullWindow();
    display.firstPage();
    display.fillScreen(GxEPD_WHITE);
    do {
      uint32_t rowPosition = flip ? (height - h) * rowSize : 0;
      for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) {
        uint8_t gray = 0;
        uint8_t level = 0;
        uint16_t color = GxEPD_WHITE;
        uint32_t val;
#ifdef SHOW_GRAY_SCALE
        Serial.println("");
#endif
        for (uint16_t col = 0; col < w; col++) { // for each pixel
          red = data[rowPosition + col * 3];
          if(red > 255) {
            Serial.print("red: ");Serial.println(red);
            red = 255;
          }
          color = gamme_curve[red];
          uint16_t yrow = y + (flip ? h - row - 1 : row);
          display.drawPixel(x + col, yrow, color);

#ifdef SHOW_GRAY_SCALE
          switch (color) {
            case GxEPD_WHITE:
              Serial.print("W");
              break;
            case GxEPD_LIGHTGREY:
              Serial.print("L");
              break;
            case GxEPD_DARKGREY:
              Serial.print("D");
              break;
            case GxEPD_BLACK:
              Serial.print("B");
              break;
            default:
              Serial.print("?");
              break;
          }
#endif            
        } // end pixel
      } // end line
      Serial.println("");
    } while (display.nextPage());
    Serial.print("loaded in "); Serial.print(millis() - startTime); Serial.println(" ms");
  }
  if (!valid) {
    Serial.println("bitmap format not handled.");
  }
}

void show_memory(char arr[]) {
  Serial.println(arr);
  Serial.print("Total heap: ");Serial.println(ESP.getHeapSize());
  Serial.print("Free heap: ");Serial.println(ESP.getFreeHeap());
  Serial.print("Total PSRAM: ");Serial.println(ESP.getPsramSize());
  Serial.print("Free PSRAM: ");Serial.println(ESP.getFreePsram());
}

uint8_t color888ToGray256(uint8_t r, uint8_t g, uint8_t b) {
  return ((r + g + b) / 3);
}

void colorGray256To888(uint8_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = color;
  g = color;
  b = color;
}

bool colorGray256ToBool(uint8_t gs) {
  return gs >> 7;
}

void quantize(uint8_t quant_bits, uint8_t &r, uint8_t &g, uint8_t &b){
  uint8_t quant_step = 255 / ((1 << quant_bits) - 1);
  uint8_t shifter = 8 - quant_bits;
  r = ((uint16_t)r >> shifter) * quant_step;
  g = ((uint16_t)g >> shifter) * quant_step;
  b = ((uint16_t)b >> shifter) * quant_step;
}

void quantize_BW(uint8_t &c){
  c = (int8_t)c >> 7;
}

bool is_2s_pow(uint16_t number) {
    return !((number) & ((number) - 1));
}

uint8_t twos_power(uint16_t number){		// returns the position of the highest (MS) '1' in a power of 2 number
	uint8_t l2 = 0;
	while(number >>= 1){
	  l2++;
	}
	return l2;
}

uint8_t clamp(int16_t v, uint8_t min, uint8_t max){	// Clamps values between a range inside [0 : 255] ; substitute "uint8_t min, uint8_t max" with "uint16_t ..." to extend this range.
  uint16_t t = (v) < min ? min : (v);
  return t > max ? max : t;
}

uint32_t index(uint32_t x, int32_t y, uint8_t bpp){		// ONLY for byte-aligned pixels (so monochrome or, generally speaking, single-byte color such as RGB332 format)
  uint32_t result = (x+y)*bpp; 
  if (result >= 921600 || result < 0) {
   Serial.printf("Index: %d (%dx%d) \n", result, x, y);
  }
  return result;
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