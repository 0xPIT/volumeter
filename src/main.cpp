#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <PolledTimeout.h>

#include <time.h>
#include <sys/time.h>
#include <sntp.h>
#include <TZ.h>
#define TZ TZ_Europe_Berlin

#include <U8g2lib.h> // U8G2_SSD1306_128X32_[UNIVISION|WINSTAR]_[1|2|F]_HW_I2C
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);
uint16_t displayW;
uint16_t displayH;

// #define SPARKFUN_TOF 1
// #define ADAFRUIT_TOF 1
// #define ADAFRUIT_VL6180X_TOF 1
#define POLULO_VL6180X_TOF 1
// #define SHARP_TOF 1

#ifdef SHARP_TOF
#include "Sharp-GP2Y0E0xx.h"
using SensorDriver = GP2Y0E0xx::Sensor;
#endif

#ifdef SPARKFUN_TOF
#include "SparkFun_VL53L1X.h"
using SensorDriver = SFEVL53L1X;
#endif

#ifdef ADAFRUIT_TOF
#include <Adafruit_VL53L0X.h>
using SensorDriver = Adafruit_VL53L0X;
#endif

#ifdef ADAFRUIT_VL6180X_TOF
#include <Adafruit_VL6180X.h>
using SensorDriver = Adafruit_VL6180X;
#endif

#ifdef POLULO_VL6180X_TOF
  #include <VL6180X.h>
  using SensorDriver = VL6180X;
#endif

SensorDriver distanceSensor;

#include "Filters.h"
WaterLevelFilter<5> sensorFilter(0.15);

#include "Tank.h"
#include "machine.h"
WaterTank tank(Machines::DefaultTank);


void clearArea(int x, int y, int w, int h) {
    display.setDrawColor(0); 
    display.drawBox(x, y, w, h); 
    display.setDrawColor(1);
}

void drawProgressbar(int x, int y, int w, int h, float value) {
    const uint8_t space = 2;
    if (value > 1.0) value = 1.0;
    clearArea(x, y, w, h);
    display.drawFrame(x, y, w, h);
    uint16_t barWidth = static_cast<uint16_t>((w - 2 * space) * value);
    display.drawBox(x + space, y + space, barWidth, h - 2 * space);
    display.updateDisplay();
}

void centerProgressbar(float value, uint16_t height = 8) {
  const uint16_t border = 2;
  drawProgressbar(border, displayH / 2 - height / 2, displayW - border, height, value);
}

const char* startMessage = "Starting";

void setupDisplay() {  
  display.begin();
  display.clear();
  display.setFont(u8g2_font_profont11_mf);
  display.drawStr(0, 8, startMessage);
  display.updateDisplay();
  displayH = display.getDisplayHeight();
  displayW = display.getDisplayWidth();
}

void setupClock() {
  time_t rtc = 1588617632;
  timeval tv = { rtc, 0 };
  settimeofday(&tv, nullptr);
  configTime(TZ, "pool.ntp.org");
}

String hostname = "espdist-initial";

String& setHostname(String name = "") {
  hostname = name;
  if (hostname == "") {
    hostname = "espdist-";
    hostname += String(ESP.getChipId(), HEX);
  }
  WiFi.hostname(hostname);
  return hostname;
}

void setupOTAUpdate(const String& hostname) {
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.begin(true); // true = with mDNS
  
  ArduinoOTA.onStart([]() {
    display.clear();
    display.setFont(u8g2_font_profont11_mf);
    display.setCursor(0, 8);
    display.println("Updating firmware...");
    centerProgressbar(0.0f);
  });

  ArduinoOTA.onEnd([]() {
    display.clear();
    display.setCursor(0, 8);
    display.println("Update complete.");
    display.setCursor(0, 18);
    display.print("Rebooting...");
    display.updateDisplay();
    ESP.restart();
  });

  ArduinoOTA.onProgress([](uint32_t progress, uint32_t total) {
    char buf[40];
    clearArea(0, displayH - 8, displayW, 8);
    display.setCursor(0, displayH);
    sprintf(buf, "%dk/%dk\r", progress / 1024, total / 1024);
    display.print(buf);
    centerProgressbar((float)(progress / (total / 100)) / 100);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    display.clear();
    display.setCursor(0, 8);
    display.println("Update failed.");
    display.setCursor(0, 18);
    display.print("Rebooting...");
    display.updateDisplay();
    ESP.restart();
  });
}

void setupWifi() {
  Serial.println("connecting to WiFi...");
  setHostname();
  WiFi.mode(WIFI_STA);
  WiFi.begin("pit", "YOUR_PASSWORD");

  const String indicator[6] = { ".  ", ".. ", "...", " ..", "  .", "   " };
  uint8_t progIdx = 0;
  display.setFont(u8g2_font_profont11_mf);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    display.drawStr(display.getStrWidth(startMessage) + 2, 8, indicator[progIdx].c_str());
    if (++progIdx % 5 == 0) progIdx = 0;
    display.updateDisplay();
    delay(100);
  }

  Serial.println(hostname);
  Serial.println(WiFi.localIP());

  MDNS.begin(hostname);
  MDNS.addService("http", "tcp", 80);
}

#ifdef USE_INTERRUPT
const int sensorInterruptPin = D8;
volatile uint8_t measurementPending = 0;

void IRAM_ATTR measurementAvailable() {
  measurementPending = 1;
}

void setupSensorInterrupt() {
  pinMode(sensorInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorInterruptPin), measurementAvailable, FALLING);
}
#endif


uint8_t sensorScaled = 2;

void setupSensor() {
  Wire.begin();

  Serial.print("Setup ToF sensor ");

#ifdef SHARP_TOF
  distanceSensor.begin();
#endif

#ifdef ADAFRUIT_TOF
  if (!distanceSensor.begin(VL53L0X_I2C_ADDR, true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY)) {
    Serial.println("failed to initialize [halt]");
    while (1) ;
  }

  distanceSensor.startRangeContinuous();
#endif

#ifdef SPARKFUN_TOF
  // default sensor seems to be at 0x52
  if (distanceSensor.begin() != 0) {
    Serial.println("failed to initialize [halt]");
    while (1) ;
  }

  distanceSensor.setDistanceModeShort();
  // distanceSensor.getOffset();
  // distanceSensor.setOffset(-30);
	distanceSensor.setIntermeasurementPeriod(400);
  distanceSensor.setTimingBudgetInMs(350);
  distanceSensor.startRanging();
#endif

#ifdef ADAFRUIT_VL6180X_TOF
  distanceSensor.begin();
#endif

#ifdef POLULO_VL6180X_TOF
  distanceSensor.init();
  distanceSensor.configureDefault();
  distanceSensor.setScaling(sensorScaled);
  distanceSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  distanceSensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  distanceSensor.setTimeout(500);
  distanceSensor.stopContinuous();

  #ifdef USE_INTERRUPT
    setupSensorInterrupt();
    // enable interrupt output on GPIO1
    distanceSensor.writeReg(VL6180X::SYSTEM__MODE_GPIO1, 0x10);
    // clear any existing interrupts
    distanceSensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x03);    
  #endif

  delay(300);
  distanceSensor.startInterleavedContinuous(100);
#endif

  Serial.printf(" done.");
}



uint16_t readSensor() {
  uint16_t rawDistance = 0;
    
  #if SPARKFUN_TOF
    while (!distanceSensor.checkForDataReady()) {
      delay(10);
    }

    rawDistance = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
  #endif

  #ifdef ADAFRUIT_TOF
    while (!distanceSensor.isRangeComplete()) {
      delay(10);
    }
    rawDistance = distanceSensor.readRangeResult();
  #endif

  #ifdef SHARP_TOF
    distanceSensor.getDistance();
  #endif

  #ifdef ADAFRUIT_VL6180X_TOF
    // float lux = distanceSensor.readLux(VL6180X_ALS_GAIN_1);
    // Serial.print("Lux: "); Serial.println(lux);
    rawDistance = distanceSensor.readRange();
    uint8_t status = distanceSensor.readRangeStatus();
    // if (status == VL6180X_ERROR_NONE) {
    //   Serial.print("Range: "); Serial.println(rawDistance);
    // }
  #endif

  #ifdef POLULO_VL6180X_TOF
    rawDistance = distanceSensor.readRangeContinuousMillimeters() * sensorScaled;

    // if (sensorScaled == 1 && rawDistance > 150) {
    //     sensorScaled = 2;
    //     distanceSensor.setScaling(sensorScaled);
    // }
    
    // if (sensorScaled == 2 && rawDistance < 150) {
    //     sensorScaled = 1;
    //     distanceSensor.setScaling(sensorScaled);
    // }

    if (distanceSensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  #endif

  return rawDistance;
}

uint8_t mapToSteps(uint16_t input, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max, uint8_t step = 10) {
    if (in_min >= in_max) return out_min;
    input = max(min(input, in_max), in_min);    
    float normalized = (float)(input - in_min) / (in_max - in_min); // normalize (0.0 to 1.0)
    float scaled = normalized * (out_max - out_min) + out_min;      // Scale to output range
    int mapped = roundf(scaled / step) * step;                      // Round to nearest step
    return (uint8_t)max(min(mapped, (int)out_max), (int)out_min);
}

uint16_t lastFillPercentage = 0xff;

auto onDistanceChanged = [](int16_t distance) -> void {
    distance -= tank.sensorOffset();
    if (distance > tank.tank.h) distance = tank.tank.h;
    if (distance < 0) distance = 0;

    uint16_t filled = tank.tank.h - distance;
    uint16_t fillVolume = tank.filledVolumeTo(filled);
    uint16_t fillPercentage = tank.filledPercent(filled);

    if (fillPercentage != lastFillPercentage) {
        lastFillPercentage = fillPercentage;
        display.firstPage();
        do {
            display.setFont(u8g2_font_profont22_mf);
            display.setCursor(0, 20);
            display.printf("%2d%%", fillPercentage);
            const uint16_t percentW = 3 * display.getMaxCharWidth() + 6;
            display.setFont(u8g2_font_profont11_mf);
            display.setCursor(percentW, 22);
            display.printf("%d + %dmm", distance, filled);
            display.setCursor(percentW, 32);
            display.printf("%d of %dml", fillVolume, tank.fullVolume());
        } while (display.nextPage());
    }
};

uint16_t distance = 0;
uint16_t distancePrevious = 0;

auto measureDistance = [](const std::function<void(uint16_t&)> &onChange) -> bool {
  bool distanceChanged = false;
  distancePrevious = distance;

  uint16_t rawDistance = readSensor();
  distance = sensorFilter.filter(rawDistance);

  distanceChanged = abs(distance - distancePrevious) > 0;

  Serial.printf("raw: %d, avg: %d, pre: %d%s\n", rawDistance, distance, distancePrevious, distanceChanged ? " ***" : "");

  if (distanceChanged) {
      onChange(distance);
  }

  return distanceChanged;
};


void setup() {
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);

  setupDisplay();

  // setupWifi();
  // setupOTAUpdate(hostname);
  // setupClock();

  display.setFont(u8g2_font_profont11_mf);
  display.drawStr(0, 20, WiFi.localIP().toString().c_str());
  display.drawStr(0, 30, hostname.c_str());
  display.sendBuffer();
  delay(2000);
  display.clear();

  setupSensor();
}

void loop() {

  #ifdef USE_INTERRUPT
    if (measurementPending) {
      measurementPending = 0;
      Serial.println("measurementPending");
    }
  #endif

  measureDistance(onDistanceChanged);

  // ArduinoOTA.handle();
  // MDNS.update();
  
  delay(25);
}
