#include <Arduino.h>


#include <WiFi.h>
#include <TFT_eSPI.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>

#include "fonts/BigFont.h"
#include "fonts/SecFont.h"
#include "fonts/SmallFont.h"
#include "fonts/CrystalFont96.h"
#include "fonts/RollbloxFont.h"

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
TFT_eSprite needle = TFT_eSprite(&tft);

#define NEEDLE_LENGTH 25  // Visible length
#define NEEDLE_RADIUS 80  // Radius at tip

static long prevPos = 0;

static int currentScreen = 0;

static int NUMBER_OF_SCREENS = 7;
static int CLICKS_PER_SCREEN = 4;

#define DEG2RAD 0.0174532925

static uint16_t uiCounter = 0;

static bool startup = true;

int read_touch(int *x, int *y);
int i2c_read(uint16_t addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length);
int i2c_write(uint8_t addr, uint8_t reg_addr, const uint8_t *reg_data, uint32_t length);
void encoder_irq();

Arduino_DataBus *bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED /* DC */, 1 /* CS */,
    46 /* SCK */, 0 /* MOSI */, GFX_NOT_DEFINED /* MISO */);

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    2 /* DE */, 42/* VSYNC */, 3 /* HSYNC */, 45 /* PCLK */,
    4 /* R0 */, 41 /* R1 */, 5 /* R2 */, 40 /* R3 */, 6 /* R4 */,
    39 /* G0/P22 */, 7 /* G1/P23 */, 47 /* G2/P24 */, 8 /* G3/P25 */, 48 /* G4/P26 */, 9 /* G5 */,
    11 /* B0 */, 15 /* B1 */, 12 /* B2 */, 16 /* B3 */, 21 /* B4 */,
    1 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    1 /* vsync_polarity */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);


Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST */, st7701_type5_init_operations, sizeof(st7701_type5_init_operations));



#define I2C_SDA_PIN 17
#define I2C_SCL_PIN 18
#define TOUCH_RST -1 // 38
#define TOUCH_IRQ -1 // 0

#define TFT_BL 38
#define BUTTON_PIN 14
#define ENCODER_CLK 13 // CLK
#define ENCODER_DT 10  // DT

int counter = 0;
int State;
int old_State;

int move_flag = 0;
int button_flag = 0;
int flesh_flag = 1;

void pin_init()
{
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  old_State = digitalRead(ENCODER_CLK);

  attachInterrupt(ENCODER_CLK, encoder_irq, CHANGE);
}

void encoder_irq()
{
  State = digitalRead(ENCODER_CLK);
  if (State != old_State)
  {
    if (digitalRead(ENCODER_DT) == State)
    {
      counter++;
    }
    else
    {
      counter--;
    }
  }
  old_State = State; // the first position was changed
  move_flag = 1;
}

static void screenTypeOneSetup(const char *title)
{
    sprite.createSprite(480, 480);
    sprite.fillSprite(0xABE1);

    sprite.setTextColor(WHITE);
    sprite.setTextDatum(MC_DATUM);
    //sprite.loadFont(&fonts::Orbitron_Light_32); //XXX
    sprite.loadFont(secFont);
    sprite.drawString(title, 480 / 2, 20);
    
}

static void screenTypeOneSetupSubTitle(const char *title, const char *subTitle)
{
    sprite.createSprite(480, 480);
    sprite.fillSprite(0x010B);
    sprite.setTextColor(WHITE);
    sprite.setTextDatum(MC_DATUM);
    sprite.loadFont(secFont);
    sprite.setTextSize(3);
    sprite.drawString(title, 480 / 2, 20);
    sprite.loadFont(rollbloxFont);    
    sprite.fillRoundRect(80, 45, 400, 45, 8, 0xAEDE);
    uint32_t oldTextColor = sprite.textcolor;
    sprite.setTextSize(6);
    sprite.setTextColor(0x010B);
    sprite.drawString(subTitle, 480 / 2, 65);
    sprite.setTextColor(oldTextColor);
}

static void screenTypeOne(const char *title, long digits)
{
    char tempString[50];
    screenTypeOneSetup(title);
    snprintf(tempString, sizeof(tempString), "%d", digits);
    sprite.drawString(tempString, 480 / 2, 480 / 2);
    gfx->draw16bitBeRGBBitmap(0,0,(uint16_t*)sprite.getPointer(),480,480);    
}

static void screenTypeOne(const char *title, float digits)
{
    char tempString[50];
    screenTypeOneSetup(title);
    snprintf(tempString, sizeof(tempString), "%4.1f", digits);
    sprite.drawString(tempString, 480 / 2, 480 / 2);
    gfx->draw16bitBeRGBBitmap(0,0,(uint16_t*)sprite.getPointer(),480,480);    
}

void screenTypeOneCharString(const char *title, String text)
{
    char tempString[50];
    screenTypeOneSetup(title);
    snprintf(tempString, sizeof(tempString), "%s", text);
    sprite.drawString(tempString, 480 / 2, 480 / 2);
    gfx->draw16bitBeRGBBitmap(0,0,(uint16_t*)sprite.getPointer(),480,480);    
}

void screenTypeOneDigitString(const char *title, String text)
{
    char tempString[50];
    screenTypeOneSetup(title);
    snprintf(tempString, sizeof(tempString), "%s", text);
    //sprite.setFreeFont(Font7);//XXX
    sprite.drawString(tempString, 480 / 2, 480 / 2, 4);
    gfx->draw16bitBeRGBBitmap(0,0,(uint16_t*)sprite.getPointer(),480,480);    
}


/* used for clock and others*/
void screenTypeOneDigitStringSubTitle(const char *title, const char *subTitle, String text)
{
    char tempString[50];
    screenTypeOneSetupSubTitle(title, subTitle);
    snprintf(tempString, sizeof(tempString), "%s", text);
    sprite.loadFont(crystalFont96); //from volos
    sprite.drawString(tempString, 480 / 2, 480 / 2);
    gfx->draw16bitBeRGBBitmap(0,0,(uint16_t*)sprite.getPointer(),480,480);    

    sprite.unloadFont();
}

static void screenClock()
{  
  char stringToPass[40];
  snprintf(stringToPass, sizeof(stringToPass), "%2.2d:%2.2d:%2.2d ", 12,22, counter);
  screenTypeOneDigitStringSubTitle("Clock", "TORQ", stringToPass);
}

void createNeedle(void)
{
  needle.setColorDepth(8);
  needle.createSprite(11, NEEDLE_LENGTH);  // create the needle Sprite 11 pixels wide by 30 high

  needle.fillSprite(TFT_BLACK); // Fill with black

  // Define needle pivot point
  uint16_t piv_x = needle.width() / 2;   // pivot x in Sprite (middle)
  uint16_t piv_y = NEEDLE_RADIUS;        // pivot y in Sprite
  needle.setPivot(piv_x, piv_y);         // Set pivot point in this Sprite

  // Draw the red needle with a yellow tip
  // Keep needle tip 1 pixel inside dial circle to avoid leaving stray pixels
  needle.fillRect(piv_x - 1, 2, 3, NEEDLE_LENGTH, TFT_RED);
  needle.fillRect(piv_x - 1, 2, 3, 5, TFT_YELLOW);
  gfx->draw16bitBeRGBBitmap(0,0,(uint16_t*)needle.getPointer(),480,480);    
}

void plotNeedle(int angle, byte ms_delay)
{
  static int16_t old_angle = -120; // Starts at -120 degrees

  if (angle < 0) angle = 0; // Limit angle to emulate needle end stops
  if (angle > 240) angle = 240;

  angle -= 120; // Starts at -120 degrees

  // Move the needle until new angle reached
  while (angle != old_angle) {

    if (old_angle < angle) old_angle++;
    else old_angle--;

    // Draw the needle in the new postion
    needle.pushRotated(old_angle);

    // Slow needle down slightly as it approaches new postion
    if (abs(old_angle - angle) < 10) ms_delay += ms_delay / 5;

    // Wait before next update
    delay(ms_delay);
  }
}

void setup() {
  USBSerial.begin(115200);
  gfx->begin();
  gfx->fillScreen(BLACK);

  pin_init();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop() {
  screenClock();

    if (digitalRead(BUTTON_PIN) == 0)
  {
    if (button_flag != 1)
    {
      button_flag = 1;
      flesh_flag = 1;
    }

    USBSerial.println("Button Press");
  }
  else
  {
    if (button_flag != 0)
    {
      button_flag = 0;
      flesh_flag = 1;
    }
  }

  if (move_flag == 1)
  {
    USBSerial.print("Position: ");
    USBSerial.println(counter);
    move_flag = 0;
    flesh_flag = 1;
  }
}

