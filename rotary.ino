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

RotaryEncoder encoder(ENCODERA, ENCODERB, 0, 2399);

void updatePositionISR() { encoder.updatePosition(); }

void setup() {
  Serial.begin(115200);
  encoder.begin();
}

void loop() {
  if (encoder.hasChanged()) {
    Serial.print(encoder.getDirection() == CW ? "CW " : "CCW");
    Serial.print(" ");
    Serial.println(encoder.getPosition());
  }
}
