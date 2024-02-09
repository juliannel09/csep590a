/**
* A2: Wearable Step Tracker
* author@ jlin09
* CSE P 590 A 
* Assumption: The tracker will be worn on the wrist like a watch
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "config.h" // config.h file not included due to credentials -- should be similar to the example with the IO library

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D  // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

const int BATTERY_PIN = A13;

// Instantiate accelerometer 
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int16_t x, y, z;
uint16_t textWidth, textHeight;

// Instantiate SSD1306 driver display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// set up the accerlerometer magnitude feed
// data passed to Adafruit IO will be the peak value of detected steps
AdafruitIO_Feed *stepsFeed = io.feed("numSteps");

const int BATTERY_H = 12;
const int BATTERY_W = 24;
const unsigned char small_battery [] = {
  0xff, 0xff, 0xfe, 
  0x80, 0x00, 0x02, 
  0x80, 0x00, 0x03, 
  0x80, 0x00, 0x03, 
  0x80, 0x00, 0x03, 
  0x80, 0x00, 0x03,
  0x80, 0x00, 0x03, 
  0x80, 0x00, 0x03,
  0x80, 0x00, 0x03, 
  0x80, 0x00, 0x03,
  0x80, 0x00, 0x02, 
  0xff, 0xff, 0xfe
};
const int POWER_H = 8;
const int POWER_W = 4;

// Right where we can start drawing the power rectangles
const int BATT_X = 1;
const int BATT_Y = 1;
const int OFFSET = 1; // Spacing in between power bars

// Snowflake display constants
const int SNOWFLAKE_H = 16;
const int SNOWFLAKE_W = 16;
const int MAX_NUM_FLAKES = 15;
const int XPOS = 0;
const int YPOS = 1;

// Snowflake bitmap
static const unsigned char snowflake[] = { 
  0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 
};
int8_t flakes[MAX_NUM_FLAKES][3];

// Arduino IO constants
unsigned long lastUploadTimestamp = 0;
int lastUploadedVal = -1;
const unsigned long MAX_ELAPSED_TIME_BETWEEN_UPLOADS_MS = 10000; // max upload rate on free tier is 30/minute 
const unsigned long MIN_ELAPSED_TIME_BETWEEN_UPLOADS_MS = 2000; // don't set this lower than 2000 (2 secs) given upload rate limit

// Algorithm constants and variables
const int FILTER_WINDOW_SIZE = 3;
const int DELAY_MS = 10;

float power;
int buffer;
double accel[FILTER_WINDOW_SIZE] = {};
int curAccelIndex = 0;
int accelTotal = 0;

int minPeakDist = 500;
int minPeakHeight = 1500;
int maxPeakHeight = 5000;
int lastPeakTime = -1;
int totalPeaks = 0;

// This setup code is based on the acceleration demo example in the LIS3DH library
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (!lis.begin(0x18)) {  // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // Initialize the display. If it fails, print failure to Serial
  // and enter an infinite loop
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  Serial.println("SSD1306 found!");

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  display.clearDisplay();

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }

  buffer = 1/(DELAY_MS * 0.001); // buffer size as function of sample rate/delay time
}

void loop() {
  unsigned long currentTimestamp = millis();
  io.run(); // keep client connected to Adafruit IO
  if (currentTimestamp - lastUploadTimestamp > MIN_ELAPSED_TIME_BETWEEN_UPLOADS_MS) {
    stepsFeed->save(totalPeaks);
    lastUploadTimestamp = currentTimestamp;
  }

  power = analogRead(BATTERY_PIN)*6.6; // x2 bc voltage divided, x3.3 bc ref voltage of ESP32 is 3.3V
  power /= 4095; // convert to voltage - max input is 4095
  displayBattery();

  double accelMag[buffer] = {};
  int bufferTotal = 0;
  for (int i=0; i<buffer; i++) {
    lis.read(); 
    double cur = sqrt(pow(lis.x, 2) + pow(lis.y, 2) + pow(lis.z, 2)); // absolute magnitude across three axis
    cur = smoothingFilter(cur);
    accelMag[i] = cur;
    bufferTotal += cur;
  }
  double bufferAvg = bufferTotal / buffer;
  for (int i=0; i<buffer; i++) {
    accelMag[i] -= bufferAvg; // normalize the acceleration
  }

  peakDetection(accelMag);
  delay(DELAY_MS);
}

double smoothingFilter(double currentAccelMag) {
  accelTotal -= accel[curAccelIndex];
  accel[curAccelIndex] = currentAccelMag;
  accelTotal += currentAccelMag;
  curAccelIndex = (curAccelIndex + 1) % FILTER_WINDOW_SIZE;
  return accelTotal / FILTER_WINDOW_SIZE;
}

// This peak detection code is based on: 
// https://makeabilitylab.github.io/physcomp/signals/StepTracker/index.html
void peakDetection(double accelMag[]) {
  for (int i = 0; i < buffer; i++) {
    double forwardSlope = accelMag[i+1] - accelMag[i];
    double backwardSlope;
    if (i == 0) {
      backwardSlope = accelMag[i] - accelMag[buffer-1];
    } else {
      backwardSlope = accelMag[i] - accelMag[i-1];
    }
    if (forwardSlope < 0 && backwardSlope > 0) {
      int timeNow = millis();
      double peakVal = accelMag[i];
      if (peakVal >= minPeakHeight && peakVal <= maxPeakHeight) {
        int prevPeakTimeDiff = timeNow - lastPeakTime;
        if (lastPeakTime == -1 || prevPeakTimeDiff >= minPeakDist) {
          lastPeakTime = timeNow;
          totalPeaks += 1;
          Serial.println(peakVal);
          Serial.print("It's a step!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: ");
          Serial.println(totalPeaks);
          Serial.print("peakVal: ");
          Serial.println(peakVal);
          Serial.print("peakTimeDiff: ");
          Serial.println(prevPeakTimeDiff);
          if (totalPeaks % MAX_NUM_FLAKES == 1) {
            display.clearDisplay();
            displayBattery();
          }
          drawSnowflake();
        }
      } 
    }
  }
}

void displayBattery() {
  // Serial.println(power);
  display.drawRect(0, 0, BATTERY_W, BATTERY_H, BLACK);
  display.drawBitmap(0, 0, small_battery, BATTERY_W, BATTERY_H, WHITE);

  int powerMode;
  if (power >= 3.8) {
    powerMode = 4;
  } else if (power >= 3.6) {
    powerMode = 3;
  } else if (power >= 3.4) {
    powerMode = 2;
  } else if (power >= 3.3) {
    powerMode = 1;
  } else {
    powerMode = 0;
  }

  int x = BATT_X;
  int y = BATT_Y;
  y += OFFSET;
  for (int i=0; i<powerMode; i++) {
    x += OFFSET;
    display.drawRect(x, y, POWER_W, POWER_H, WHITE);
    display.fillRect(x, y, POWER_W, POWER_H, WHITE);
    x += POWER_W;
  }

  display.display();
}

void drawSnowflake() {
  int8_t flake[2];

  flake[XPOS] = random(24, display.width() - SNOWFLAKE_W);
  flake[YPOS] = random(0, display.height() - SNOWFLAKE_H);
  Serial.print(F("x: "));
  Serial.print(flake[XPOS], DEC);
  Serial.print(F(" y: "));
  Serial.print(flake[YPOS], DEC);

  display.drawBitmap(flake[XPOS], flake[YPOS], snowflake, SNOWFLAKE_W, SNOWFLAKE_H, SSD1306_WHITE);
  display.display();
}
