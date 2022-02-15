/*
    Smartglove for cyclists: https://www.instructables.com/member/Matlek/
    This code is made for an ESP32 microcontroller, to control an LED matrix through BLE.
    It is a mix of the following codes, after a few modifications:
    -"BLE_Write" example from the "BLE ESP32 ARDUINO" library.
    -"MatrixGFXDemo64" example from the "FastLED NeoMatrix" library.
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
String gestureValue = "0";
int gestureNb = 0;
int old_gestureNb = 0;
BLEServer *pServer = NULL;
bool deviceConnected = false;
unsigned long previousMillis = 0;
const long interval = 1000;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "e4297ee0-8c88-11ea-bc55-0242ac130003"
#define CHARACTERISTIC_UUID "e4297ee1-8c88-11ea-bc55-0242ac130003"
#define DISABLE_WHITE
#include <Adafruit_GFX.h>
#include <FastLED_NeoMatrix.h>
#include <FastLED.h>
// Allow temporaly dithering, does not work with ESP32 right now
#ifndef ESP32
#define delay FastLED.delay
#endif
#define PIN 33
#define BRIGHTNESS 64
#define mw 20
#define mh 8
#define NUMMATRIX (mw*mh)
CRGB leds[NUMMATRIX];
// Define matrix width and height.
FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(leds, mw, mh,
    NEO_MATRIX_BOTTOM     + NEO_MATRIX_RIGHT +
    NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG);

void matrix_show() {
  matrix->show();
}

// This could also be defined as matrix->color(255,0,0) but those defines
// are meant to work for adafruit_gfx backends that are lacking color()
#define LED_BLACK    0

#define LED_RED_VERYLOW   (3 <<  11)
#define LED_RED_LOW     (7 <<  11)
#define LED_RED_MEDIUM    (15 << 11)
#define LED_RED_HIGH    (31 << 11)

#define LED_GREEN_VERYLOW (1 <<  5)
#define LED_GREEN_LOW     (15 << 5)
#define LED_GREEN_MEDIUM  (31 << 5)
#define LED_GREEN_HIGH    (63 << 5)

#define LED_BLUE_VERYLOW  3
#define LED_BLUE_LOW    7
#define LED_BLUE_MEDIUM   15
#define LED_BLUE_HIGH     31

#define LED_ORANGE_VERYLOW  (LED_RED_VERYLOW + LED_GREEN_VERYLOW)
#define LED_ORANGE_LOW    (LED_RED_LOW     + LED_GREEN_LOW)
#define LED_ORANGE_MEDIUM (LED_RED_MEDIUM  + LED_GREEN_MEDIUM)
#define LED_ORANGE_HIGH   (LED_RED_HIGH    + LED_GREEN_HIGH)

#define LED_PURPLE_VERYLOW  (LED_RED_VERYLOW + LED_BLUE_VERYLOW)
#define LED_PURPLE_LOW    (LED_RED_LOW     + LED_BLUE_LOW)
#define LED_PURPLE_MEDIUM (LED_RED_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_PURPLE_HIGH   (LED_RED_HIGH    + LED_BLUE_HIGH)

#define LED_CYAN_VERYLOW  (LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_CYAN_LOW    (LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_CYAN_MEDIUM   (LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_CYAN_HIGH   (LED_GREEN_HIGH    + LED_BLUE_HIGH)

#define LED_WHITE_VERYLOW (LED_RED_VERYLOW + LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_WHITE_LOW   (LED_RED_LOW     + LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_WHITE_MEDIUM  (LED_RED_MEDIUM  + LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_WHITE_HIGH    (LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

void matrix_clear() {
  // clear does not work properly with multiple matrices connected via parallel inputs
  memset(leds, 0, sizeof(leds));
}

void ScrollText_Left() {
  matrix_clear();
  matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
  matrix->setTextSize(1);
  matrix->setRotation(0);
  for (int8_t x = 0; x >= -60; x--) {
    yield();
    matrix_clear();
    matrix->setCursor(x, 0);
    matrix->setTextColor(LED_ORANGE_HIGH);
    matrix->print("<<<<<<<<<<<<<<");
    matrix_show();
    delay(50);
  }
  matrix->setCursor(0, 0);
}

void ScrollText_Merci() {
  matrix_clear();
  matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
  matrix->setTextSize(1);
  matrix->setRotation(0);
  for (int8_t x = 20; x >= -30; x--) {
    yield();
    matrix_clear();
    matrix->setCursor(x, 0);
    matrix->setTextColor(LED_PURPLE_HIGH);
    matrix->print("Merci");
    matrix_show();
    delay(70);
  }
  matrix->setCursor(0, 0);
}

void ScrollText_Hello() {
  matrix_clear();
  matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
  matrix->setTextSize(1);
  matrix->setRotation(0);
  for (int8_t x = 20; x >= -30; x--) {
    yield();
    matrix_clear();
    matrix->setCursor(x, 0);
    matrix->setTextColor(LED_CYAN_HIGH);
    matrix->print("Hello!");
    matrix_show();
    delay(70);
  }
  matrix->setCursor(0, 0);
}

void ScrollText_Right() {
  matrix_clear();
  matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
  matrix->setTextSize(1);
  matrix->setRotation(0);
  for (int8_t x = -60; x <= 0 ; x++) {
    yield();
    matrix_clear();
    matrix->setCursor(x, 0);
    matrix->setTextColor(LED_ORANGE_HIGH);
    matrix->print(">>>>>>>>>>>>>>");
    matrix_show();
    delay(50);
  }
  matrix->setCursor(0, 0);
}

void ScrollText_Wait() {
  matrix_clear();
  matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
  matrix->setTextSize(1);
  matrix->setRotation(0);
  yield();
  matrix_clear();
  matrix->setCursor(2, 0);
  matrix->setTextColor(LED_ORANGE_HIGH);
  matrix->print("o");
  matrix_show();
  delay(500);
  yield();
  matrix_clear();
  matrix->setCursor(2, 0);
  matrix->setTextColor(LED_ORANGE_HIGH);
  matrix->print("oo");
  matrix_show();
  delay(500);
  yield();
  matrix_clear();
  matrix->setCursor(2, 0);
  matrix->setTextColor(LED_ORANGE_HIGH);
  matrix->print("ooo");
  matrix_show();
  delay(500);
  yield();
  matrix_clear();
  matrix->setCursor(0, 0);
  matrix->setTextColor(LED_BLACK);
  matrix_show();
  delay(500);
}

void ScrollText_Stop() {
  matrix_clear();
  matrix->setTextWrap(false);  // we don't wrap text so it scrolls nicely
  matrix->setTextSize(1);
  matrix->setRotation(0);
  yield();
  matrix_clear();
  matrix->setCursor(2, 0);
  matrix->setTextColor(LED_RED_HIGH);
  matrix->print("!!!");
  matrix_show();
  delay(1000);
  yield();
  matrix_clear();
  matrix->setCursor(0, 0);
  matrix->setTextColor(LED_BLACK);
  matrix->print("STOP");
  matrix_show();
  delay(500);
}

void ScrollText_Straight() {
  uint8_t size = max(int(mw / 8), 1);
  matrix->setRotation(3);
  matrix->setTextSize(size);
  matrix->setTextColor(LED_GREEN_HIGH);
  for (int16_t x = -10; x <= 6; x++) {
    yield();
    matrix_clear();
    matrix->setCursor(x, ((mw / 2 - size * 4) + 1));
    matrix->print(">");
    matrix_show();
    delay(100 / size);
  }
  matrix->setRotation(0);
  matrix->setCursor(0, 0);
  matrix_show();
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("      deviceConnected = true;");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("      deviceConnected = false;");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    //public:
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
          gestureValue = int(value[0]);
        gestureNb++;
      }
    }
};

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, PIN>(  leds, NUMMATRIX  ).setCorrection(TypicalLEDStrip);
  delay(1000);
  matrix->begin();
  matrix->setTextWrap(false);
  matrix->setBrightness(BRIGHTNESS);
#ifndef DISABLE_WHITE
  matrix->fillScreen(LED_WHITE_HIGH);
  matrix_show();
  delay(5000);
  matrix_clear();
#endif
  BLEDevice::init("MySmartglove");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  //pCharacteristic->setValue("Hello World");
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop() {

  if (deviceConnected) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      ScrollText_Straight();
    }
      Serial.print("gestureValue : ");
      Serial.println(gestureValue);
    if (gestureNb != old_gestureNb)
    {

      if (gestureValue == "0" || gestureValue == "4") {
        ScrollText_Left();
      }
      if (gestureValue == "5") {
        ScrollText_Right();
      }
      if (gestureValue == "3") {
        ScrollText_Stop();
      }
      if (gestureValue == "2") {
        ScrollText_Merci();
      }
      old_gestureNb = gestureNb;
    }
  }
  if (!deviceConnected) {
    ScrollText_Wait();
    pServer->startAdvertising(); // restart advertising
  }
}
