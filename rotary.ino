#include "Adafruit_TinyUSB.h"

Adafruit_USBD_HID usb_hid;

uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(1))
};

#define ENCODERA 29
#define ENCODERB 28

#define CW 0
#define CCW 1

#define RANGE_MODE_WRAP 0
#define RANGE_MODE_BOUNDED 1

void updatePositionISR();

class RotaryEncoder {
private:
  int pinA;
  int pinB;
  volatile long position;
  volatile long direction;
  volatile long lastPosition;
  volatile int lastStateA;
  long minValue;
  long maxValue;
  int rangeMode;

public:
  RotaryEncoder(int encoderPinA, int encoderPinB, long minVal, long maxVal, int range_mode = RANGE_MODE_BOUNDED) {
    pinA = encoderPinA;
    pinB = encoderPinB;
    minValue = minVal;
    maxValue = maxVal;
    rangeMode = range_mode;
    position = minVal;
    direction = CW;
    lastPosition = minVal;
    lastStateA = 0;
  }

  void begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    lastStateA = digitalRead(pinA);
    attachInterrupt(digitalPinToInterrupt(pinA), updatePositionISR, CHANGE);
  }

  void updatePosition() {
    int currentStateA = digitalRead(pinA);
    int currentStateB = digitalRead(pinB);

    // Only count on rising edge of channel A for 1x resolution
    if (currentStateA != lastStateA && currentStateA == HIGH) {
      if (currentStateB == HIGH) {
        position++;
        direction = CW;
        if (position > maxValue) {
          if (rangeMode == RANGE_MODE_WRAP) {
            position = minValue;
          } else {
            position = maxValue;
          }
        }
      } else {
        position--;
        direction = CCW;
        if (position < minValue) {
          if (rangeMode == RANGE_MODE_WRAP) {
            position = maxValue;
          } else {
            position = minValue;
          }
        }
      }
    }

    lastStateA = currentStateA;
  }

  long getPosition() { return position; }

  long getDirection() { return direction; }

  bool hasChanged() {
    if (position != lastPosition) {
      lastPosition = position;
      return true;
    }
    return false;
  }
};

RotaryEncoder encoder(ENCODERA, ENCODERB, 0, 599, RANGE_MODE_WRAP);

void updatePositionISR() { encoder.updatePosition(); }

void setup() {
  Serial.begin(115200);
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();
  encoder.begin();
  
  while(!TinyUSBDevice.mounted()) delay(1);
}

unsigned long lastKeyPress = 0;
const unsigned long debounceDelay = 100;
int positionAccumulator = 0;
const int positionThreshold = 64;

void loop() {
  if (encoder.hasChanged()) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastKeyPress > debounceDelay) {
      if (encoder.getDirection() == CW) {
        positionAccumulator++;
      } else {
        positionAccumulator--;
      }
      
      if (abs(positionAccumulator) >= positionThreshold) {
        uint16_t usage_code = 0;
        
        if (positionAccumulator > 0) {
          Serial.println("Volume Up");
          usage_code = HID_USAGE_CONSUMER_VOLUME_INCREMENT;
        } else {
          Serial.println("Volume Down");
          usage_code = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
        }
        
        usb_hid.sendReport(1, &usage_code, 2);
        delay(50);
        usage_code = 0;
        usb_hid.sendReport(1, &usage_code, 2);
        
        positionAccumulator = 0;
        lastKeyPress = currentTime;
      }
    }
  }
}
