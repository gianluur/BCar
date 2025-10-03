#include <SoftwareSerial.h>

class DCMotor {
public:
  DCMotor(const uint8_t PIN_ENABLE, const uint8_t PIN_IN1, const uint8_t PIN_IN2):
    m_pinEnable(PIN_ENABLE), m_pinIn1(PIN_IN1), m_pinIn2(PIN_IN2), speedMax(255), speedMin(100) {
      pinMode(m_pinEnable, OUTPUT);
      pinMode(m_pinIn1, OUTPUT);
      pinMode(m_pinIn2, OUTPUT);
      Serial.println("DCMOTOR STARTED CORRECTLY");
    }

  enum class Direction {
    FORWARD,
    BACKWARD
  };

  void setSpeedMax() {
    m_speed = speedMax;
    analogWrite(m_pinEnable, m_speed);
  }

  void setSpeedMin() {
    m_speed = speedMin;
    analogWrite(m_pinEnable, m_speed);
  }

  void setSpeed(const uint8_t speed) {
    m_speed = constrain(speed, speedMin, speedMax);
    analogWrite(m_pinEnable, m_speed);
  }

  void move(const Direction direction){
    digitalWrite(m_pinEnable, m_speed);
    switch (direction) {
      case Direction::FORWARD:
        digitalWrite(m_pinIn1, HIGH);
        digitalWrite(m_pinIn2, LOW);
        break;
      
      case Direction::BACKWARD:
        digitalWrite(m_pinIn1, LOW);
        digitalWrite(m_pinIn2, HIGH);
        break;

      default:
        Serial.println("INVALID DIRECTION CHOICE IN DCMOTOR");
        break;
      }
      

  }

  void stop() {
    digitalWrite(m_pinIn1, LOW);
    digitalWrite(m_pinIn2, LOW);
    analogWrite(m_pinEnable, 0);
  }

private:
  const uint8_t m_pinEnable, m_pinIn1, m_pinIn2, speedMax, speedMin;
  uint8_t m_speed;
};

class Motors {
public:
  Motors(DCMotor &leftMotor, DCMotor &rightMotor): 
    m_leftMotor(leftMotor), m_rightMotor(rightMotor) {
      Serial.println("MOTORS CONNECTION ESTABLISHED");
    }

  void setSpeed(const uint8_t speed) {
    m_leftMotor.setSpeed(speed);
    m_rightMotor.setSpeed(speed);
  }

  void moveForward() {
    m_leftMotor.setSpeedMax();
    m_rightMotor.setSpeedMax();

    m_leftMotor.move(DCMotor::Direction::FORWARD);
    m_rightMotor.move(DCMotor::Direction::FORWARD);
  }

  void moveBackward() {
    m_leftMotor.setSpeedMax();
    m_rightMotor.setSpeedMax();
    
    m_leftMotor.move(DCMotor::Direction::BACKWARD);
    m_rightMotor.move(DCMotor::Direction::BACKWARD);
  }

  void turnLeft() {
    m_leftMotor.setSpeedMin();
    m_rightMotor.setSpeedMin();

    m_leftMotor.move(DCMotor::Direction::FORWARD);
    m_rightMotor.stop();
  }

  void turnRight() {
    m_leftMotor.setSpeedMin();
    m_rightMotor.setSpeedMin();

    m_rightMotor.move(DCMotor::Direction::FORWARD);
    m_leftMotor.stop();
  }

  void stop() {
    m_leftMotor.stop();
    m_rightMotor.stop();
  }

private:
  DCMotor& m_leftMotor;
  DCMotor& m_rightMotor;
};

class USSensor {
public:
  USSensor(const uint8_t PIN_TRIGGER, const uint8_t PIN_ECHO):
    m_pinTrigger(PIN_TRIGGER), m_pinEcho(PIN_ECHO), m_threshold(50) {
    pinMode(PIN_TRIGGER, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    Serial.println("ULTRASONIC SENSOR CONNECTION ESTABLISHED");
  }

  struct Value {
    int distance;
    bool detected;
  };

  Value getObstacle() {
    digitalWrite(m_pinTrigger, LOW); //Clear trigger pin
    delayMicroseconds(2);

    digitalWrite(m_pinTrigger, HIGH); // Send pulse
    delayMicroseconds(10);
    digitalWrite(m_pinTrigger, LOW);

    unsigned long duration = pulseIn(m_pinEcho, HIGH);
    int distance = (duration / 2) / 29.1; // Calculate the distance in centimeters

    return {distance, distance <= m_threshold};
  }

  void setThreshold(const int8_t threshold) {
    m_threshold = threshold;
  }

private:
  const uint8_t m_pinTrigger, m_pinEcho;
  int8_t m_threshold;

};

// class IRSensor {
// public:
//   IRSensor(const uint8_t PIN_DIGITAL, const uint8_t PIN_ANALOG):
//     m_pinDigital(PIN_DIGITAL), m_pinAnalog(PIN_ANALOG) {
//       pinMode(m_pinDigital, INPUT);
//       pinMode(m_pinAnalog, INPUT);
//       Serial.println("IR SENSOR STARTED CORRECTLY");
//     }

//     void print() const {
//       Serial.print("Analog: ");
//       Serial.print(analogRead(m_pinAnalog));
//       Serial.print(" | Digital: ");
//       Serial.println(digitalRead(m_pinDigital));
//     }

//     uint8_t getAnalogPin() const {
//       return m_pinAnalog;
//     }

// private:
//   const uint8_t m_pinDigital, m_pinAnalog;
// };

// class LineFollower {
// public:
//   LineFollower(const IRSensor IRSensorL, const IRSensor IRSensorR):
//     m_irSensorL(IRSensorL), m_irSensorR(IRSensorR) {
//       calibrate();
//   }

//   enum LinePosition {
//     ON_LINE,       // Sensor is directly on the line
//     LEFT_OF_LINE,  // Left sensor is off the line
//     RIGHT_OF_LINE, // Right sensor is off the line
//     OFF_LINE       // Both sensors are off the line
//   };

//   LinePosition getLinePosition() {
//     int leftReading = analogRead(m_irSensorL.getAnalogPin());
//     int rightReading = analogRead(m_irSensorL.getAnalogPin());

//     bool isLeftOnLine = leftReading < lineThreshold;
//     bool isRightOnLine = rightReading < lineThreshold;

//     if (isLeftOnLine && isRightOnLine) 
//       return ON_LINE;
//     else if (isLeftOnLine && !isRightOnLine) 
//       return LEFT_OF_LINE;
//     else if (!isLeftOnLine && isRightOnLine) 
//       return RIGHT_OF_LINE;
//     else
//       return OFF_LINE;
//   }
  

// private:
//   const IRSensor& m_irSensorL;
//   const IRSensor& m_irSensorR;

//   int whiteThreshold = 0;   // Analog reading for white surface
//   int blackThreshold = 0;   // Analog reading for black line
//   int lineThreshold = 0;    // Mid-point threshold for line detection

//   void calibrate(){
//       Serial.println("IR SENSOR CALIBRATION STARTED!");

//       const uint8_t PIN_ANALOG_L = m_irSensorL.getAnalogPin();
//       const uint8_t PIN_ANALOG_R = m_irSensorL.getAnalogPin();

//       const int calibrationSamples = 100;
//       int whiteSum = 0, blackSum = 0;

//       Serial.println("PLACE BOTH IRSENSORS ON WHITE");

//       for (int i = 0; i < calibrationSamples; i++) {
//         whiteSum += analogRead(PIN_ANALOG_L);
//         whiteSum += analogRead(PIN_ANALOG_R);
//         delay(10);
//       }

//       whiteThreshold = whiteSum / (calibrationSamples * 2);

//       Serial.println("WHITE COLOR CALIBRATION TERMINATED");
//       Serial.println("PLACE BOTH SENSOR ON BLACK");

//       for (int i = 0; i < calibrationSamples; i++) {
//         blackSum += analogRead(PIN_ANALOG_L);
//         blackSum += analogRead(PIN_ANALOG_R);
//         delay(10);
//       }

//       blackThreshold = blackSum / (calibrationSamples * 2);
//       lineThreshold = (whiteThreshold + blackThreshold) / 2;  
//       Serial.println("CALIBRATION TERMINATED");
//   }
// };

class ColorSensor {
public:
  ColorSensor(const uint8_t S0, const uint8_t S1, const uint8_t S2, const uint8_t S3, const uint8_t OUT):
    m_S0(S0), m_S1(S1), m_S2(S2), m_S3(S3), m_OUT(OUT) {
      pinMode(S0,OUTPUT);
      pinMode(S1,OUTPUT);
      pinMode(m_S2, OUTPUT);
      pinMode(m_S3, OUTPUT);
      pinMode(OUT, INPUT);
      
      digitalWrite(m_S0, HIGH);
      digitalWrite(m_S1, HIGH);
      Serial.println("COLOR SENSOR CONNECTION ESTABLISHED");
    }

  void getColor() {
    int redFrequency, greenFrequency, blueFrequency;

    digitalWrite(m_S2, LOW);
    digitalWrite(m_S3, LOW);
    redFrequency = pulseIn(m_OUT, LOW);
    
    // Read Green
    digitalWrite(m_S2, HIGH);
    digitalWrite(m_S3, HIGH);
    greenFrequency = pulseIn(m_OUT, LOW);
    
    // Read Blue
    digitalWrite(m_S2, LOW);
    digitalWrite(m_S3, HIGH);
    blueFrequency = pulseIn(m_OUT, LOW);
    
    Serial.print("Red: ");
    Serial.print(redFrequency);
    Serial.print(" Green: ");
    Serial.print(greenFrequency);
    Serial.print(" Blue: ");
    Serial.println(blueFrequency);
  }



private:
  const uint8_t m_S0, m_S1, m_S2, m_S3, m_OUT;
  int red, green, blue;
};

DCMotor* motorA;
DCMotor* motorB;
Motors* motors;

// IRSensor* irSensorA;
// IRSensor* irSensorB;

ColorSensor* colorsensor;
USSensor* eyes;

SoftwareSerial BTSerial(0, 1); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial.println("SERIAL COMMUNICATION ESTABLISHED");

  BTSerial.begin(9600);
  Serial.println("Slave is ready...");

  motorA = new DCMotor(9, 8, 7);
  motorB = new DCMotor(3, 5, 4);
  motors = new Motors(*motorA, *motorB);

  // irSensorA = new IRSensor(2, A3);
  // irSensorB = new IRSensor(0, A4);

  colorsensor = new ColorSensor(13, 12, 11, 10, 6);
  eyes = new USSensor(A0, A1);

  motors->setSpeed(255);
  eyes->setThreshold(30);
}

void loop() {
  char command = BTSerial.read();

  switch(command){
    case 'F':
      motors->moveForward();
      Serial.println("Moving Forward");
      break;

    case 'B':
      motors->moveBackward();
      Serial.println("Moving Backwards");
      break;

    case 'L':
      motors->turnLeft();
      Serial.println("Turn left");
      break;

    case 'R':
      motors->turnRight();
      Serial.println("Turn Right");
      break;
    
    case 'S':
      motors->stop();
      Serial.println("Stop");
      break;
      
    default:
      break;
  }
}
