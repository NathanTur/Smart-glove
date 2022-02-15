/*
  Smartglove for cyclists: https://www.instructables.com/member/Matlek/
  This code is made for an Arduino Nano 33 BLE Sense board:
  it detects left hand gestures, and sends the gestures information through BLE (to another microcontroller with an LED matrix);
  It is a mix of the following codes, after a few modifications:
    -"LED" example from the "ArduinoBLE" library (Peripheral>LED).
    -"IMU_Classifier" found here: https://github.com/arduino/ArduinoTensorFlowLiteTutorials/blob/master/GestureToEmoji/ArduinoSketches/IMU_Classifier/IMU_Classifier.ino.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <TensorFlowLite.h>
#include <tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h>
#include <tensorflow/lite/experimental/micro/micro_error_reporter.h>
#include <tensorflow/lite/experimental/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>
#include "model.h"
const float gyroscopeThreshold = 300;
const int numSamples = 64;
int samplesRead = numSamples;
// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;
// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::ops::micro::AllOpsResolver tflOpsResolver;
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;
// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize];
// array to map gesture index to a name
const char* GESTURES[] = {
  "Left arm",
  "Brake",
  "Hand front rotation",
  "Hand back rotation",
  "Hand left",
  "Hand right"
};
#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))
int nbGesture = 0;
int oldNbGesture = 0;
int gestureTriggered;

// variables for button
int buttonPin = 7;
int oldButtonState = LOW;



void setup() {
  Serial.begin(115200);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  // print out the samples rates of the IMUs
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }
  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);
  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();
  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(buttonPin, INPUT);
  digitalWrite(22, HIGH);
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);

  // initialize the BLE hardware
  BLE.begin();
  Serial.println("BLE Central - LED control");

  // start scanning for peripherals
  //BLE.scanForUuid("e4297ee0-8c88-11ea-bc55-0242ac130003");
  BLE.scan();
}

void loop() {
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    if (peripheral.localName() != "MySmartglove") {
      return;
    }
    BLE.stopScan();
    // connect to the peripheral
    Serial.println("Connecting ...");

    if (peripheral.connect()) {
      Serial.println("Connected");
    } else {
      Serial.println("Failed to connect!");
      return;
    }

    // discover peripheral attributes
    Serial.println("Discovering attributes ...");
    if (peripheral.discoverAttributes()) {
      Serial.println("Attributes discovered");
    } else {
      Serial.println("Attribute discovery failed!");
      peripheral.disconnect();
      return;
    }

    // retrieve the LED characteristic
    BLECharacteristic ledCharacteristic = peripheral.characteristic("e4297ee1-8c88-11ea-bc55-0242ac130003");

    if (!ledCharacteristic) {
      Serial.println("Peripheral does not have LED characteristic!");
      peripheral.disconnect();
      return;
    } else if (!ledCharacteristic.canWrite()) {
      Serial.println("Peripheral does not have a writable LED characteristic!");
      peripheral.disconnect();
      return;
    }

    while (peripheral.connected()) {


      // read the button pin
    int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;

      if (buttonState) {
        Serial.println("button pressed");

        // button is pressed, write 0x01 to turn the LED on
        //ledCharacteristic.writeValue((byte)0x01);
      } else {
        Serial.println("button released");

        // button is released, write 0x00 to turn the LED off
        //ledCharacteristic.writeValue((byte)0x00);
      }
    }



      
      float aX, aY, aZ, gX, gY, gZ;
      // wait for significant motion
      while (samplesRead == numSamples) {
        if (IMU.gyroscopeAvailable()) {
          IMU.readGyroscope(gX, gY, gZ);
          // sum up the absolutes
          float gSum = fabs(gX) + fabs(gY) + fabs(gZ);
          // check if it's above the threshold
          if (gSum >= gyroscopeThreshold) {
            // reset the sample read count
            samplesRead = 0;
            break;
          }
        }
      }
      // check if the all the required samples have been read since
      // the last time the significant motion was detected
      oldNbGesture = nbGesture;
      while (samplesRead < numSamples) {
        // check if new acceleration AND gyroscope data is available
        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
          // read the acceleration and gyroscope data
          IMU.readAcceleration(aX, aY, aZ);
          IMU.readGyroscope(gX, gY, gZ);
          // normalize the IMU data between 0 to 1 and store in the model's
          // input tensor
          tflInputTensor->data.f[samplesRead * 6 + 0] = (aX + 4.0) / 8.0;
          tflInputTensor->data.f[samplesRead * 6 + 1] = (aY + 4.0) / 8.0;
          tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ + 4.0) / 8.0;
          tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
          tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
          tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;
          samplesRead++;
          if (samplesRead == numSamples) {
            // Run inferencing
            TfLiteStatus invokeStatus = tflInterpreter->Invoke();
            if (invokeStatus != kTfLiteOk) {
              while (1);
              return;
            }
            // Loop through the output tensor values from the model
            for (int i = 0; i < NUM_GESTURES; i++) {
              Serial.print(GESTURES[i]);
              Serial.print(": ");
              Serial.println(tflOutputTensor->data.f[i], 6);
              if ((tflOutputTensor->data.f[i]) > 0.6) {
                gestureTriggered = i;
                nbGesture++;
              }
            }
          }
        }
      }
      Serial.print("The gesture is :");
      Serial.println(GESTURES[gestureTriggered]);
      if (oldNbGesture != nbGesture) {
        if (gestureTriggered == 0) {
          ledCharacteristic.writeValue((byte)0x00);
          colors(0);
        }
        //        if (gestureTriggered == 1) {
        //          ledCharacteristic.writeValue((byte)0x01);
        //          colors(1);
        //        }
        if (gestureTriggered == 2) {
          ledCharacteristic.writeValue((byte)0x02);
          colors(2);
        }
        if (gestureTriggered == 3) {
          ledCharacteristic.writeValue((byte)0x03);
          colors(3);
        }
        if (gestureTriggered == 4) {
          ledCharacteristic.writeValue((byte)0x04);
          colors(4);
        }
        if (gestureTriggered == 5) {
          ledCharacteristic.writeValue((byte)0x05);
          colors(5);
        }
      }
    }
    // peripheral disconnected, start scanning again
    //BLE.scanForUuid("e4297ee0-8c88-11ea-bc55-0242ac130003");
    BLE.scan();
  }
}

int colors (int i) {
  if (i == 0) {
    for (int it1 = 0; it1 <= 1; it1++) {
      digitalWrite(22, LOW);
      digitalWrite(23, LOW);
      delay(500);
      digitalWrite(22, HIGH);
      digitalWrite(23, HIGH);
      delay(500);
    }
  }
  if (i == 1) {
    for (int it1 = 0; it1 <= 1; it1++) {
      digitalWrite(22, LOW);
      delay(500);
      digitalWrite(22, HIGH);
      delay(500);
    }
  }
  if (i == 2) {
    digitalWrite(23, LOW);
    delay(500);
    digitalWrite(23, HIGH);
    delay(500);
  }
  if (i == 3) {
    digitalWrite(22, LOW);
    delay(500);
    digitalWrite(22, HIGH);
  }
  if (i == 4) {
    digitalWrite(22, LOW);
    delay(500);
    digitalWrite(23, LOW);
    delay(500);
    digitalWrite(22, HIGH);
    digitalWrite(23, HIGH);
  }
  if (i == 5) {
    digitalWrite(23, LOW);
    delay(500);
    digitalWrite(22, LOW);
    delay(500);
    digitalWrite(22, HIGH);
    digitalWrite(23, HIGH);
  }
}
