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

DCMotor* motorA;
DCMotor* motorB;
Motors* motors;

SoftwareSerial BTSerial(0, 1); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial.println("SERIAL COMMUNICATION ESTABLISHED");

  BTSerial.begin(9600);
  Serial.println("Slave is ready...");

  motorA = new DCMotor(9, 8, 7);
  motorB = new DCMotor(3, 5, 4);
  motors = new Motors(*motorA, *motorB);

  motors->setSpeed(255);
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