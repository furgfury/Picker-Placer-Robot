#include <Servo.h>

//servos
Servo ARM_SERVO, ARM_SERVO2;

//CONSTANTS
//motor pins
const int LEFT_MOTOR_FORWARD = 11;
const int LEFT_MOTOR_BACKWARD = 10;
const int RIGHT_MOTOR_BACKWARD = 9;
const int RIGHT_MOTOR_FORWARD = 8;

//magnet pin
const int MAGNET = 4;

//servo pin
const int SERVO_1 = 7;
const int SERVO_2 = 6;

//servo offsets
const int SERVO_OFFSET = 140;
const int SERVO_OFFSET_WIGGLE = 0;

//arm states
const int ARM_DOWN = 132;
const int ARM_UP = 65;
const int WIGGLE_DEGREES = 10;

//delay and time
const int FIFTY_MILLISECONDS = 50;
const int FIFTH_SECOND = 200;
const int HALF_SECOND = 500;
const int ONE_SECOND = 1000;
const int ONE_AND_HALF_SECOND = 1500;
const int TWO_SECONDS = 2000;
const int TEN_SECONDS = 10000;

//magnet states
const bool ON = true;
const bool OFF = false;

//ping sensors
const int FRONT_RECIEVE_PING = 2;
const int FRONT_TRIGGER_PING = 3;
const int LEFT_TRIGGER_PING = A2;
const int LEFT_RECIEVE_PING = A3;
const int RIGHT_TRIGGER_PING = A0;
const int RIGHT_RECIEVE_PING = A1;

//sensor limits in inches
const int FRONT_LIMIT = 15;
const int LEFT_LIMIT = 10;
const int RIGHT_LIMIT = 10;

//VARIABLES
//ping sensor smoothing
long duration, dist;
int currentDistFront, currentDistLeft, currentDistRight;
int knownDistFront, knownDistLeft, knownDistRight;
int distInc;
int dir = 0;

//timing variables
long currentMillis;
long lastMillisPing = 0;
long lastMillisServo = 0;

//state variables
bool stopped = false;
bool obstacleFront, obstacleLeft, obstacleRight;

void setup() {
  Serial.begin(9600);

  //MOTORS
  //dc controller
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);

  //arm servos
  ARM_SERVO.attach(SERVO_1);
  ARM_SERVO2.attach(SERVO_2);

  //SENSORS
  //pings
  pinMode(FRONT_RECIEVE_PING, INPUT);
  pinMode(FRONT_TRIGGER_PING, OUTPUT);
  pinMode(LEFT_RECIEVE_PING, INPUT);
  pinMode(LEFT_TRIGGER_PING, OUTPUT);
  pinMode(RIGHT_RECIEVE_PING, INPUT);
  pinMode(RIGHT_TRIGGER_PING, OUTPUT);

  //magnet
  pinMode(MAGNET, OUTPUT);

  //clear sensors
  flashSensor(FRONT_RECIEVE_PING);
  flashSensor(LEFT_RECIEVE_PING);
  flashSensor(RIGHT_RECIEVE_PING);
}

void loop() {
  currentMillis = millis();

  //initialize arm and magnet state
  moveArm(ARM_DOWN);
  setElectromagnetState(ON);

  currentDistFront += getDistFromPING(FRONT_TRIGGER_PING, FRONT_RECIEVE_PING);
  currentDistLeft += getDistFromPING(LEFT_TRIGGER_PING, LEFT_RECIEVE_PING);
  currentDistRight += getDistFromPING(RIGHT_TRIGGER_PING, RIGHT_RECIEVE_PING);
  distInc++;

  //every 50 ms store ping averages smoothed
  if(currentMillis - lastMillisPing >= FIFTY_MILLISECONDS)
  {
    knownDistFront = currentDistFront/distInc;
    knownDistLeft = currentDistLeft/distInc;
    knownDistRight = currentDistRight/distInc;

    currentDistFront = 0;
    currentDistLeft = 0;
    currentDistRight = 0;

    distInc = 0;

    lastMillisPing = currentMillis;
  }

  //every 10 seconds raise arm and deposit magnets
  if(currentMillis - lastMillisServo >= TEN_SECONDS)
  {
    stop();
    moveArm(ARM_UP);
    delay(HALF_SECOND);
    setElectromagnetState(OFF);
    wiggleArm(WIGGLE_DEGREES);
    moveArm(ARM_DOWN);
    //adds half a second because timer beings counting as
    //servo is still moving down
    lastMillisServo = currentMillis + HALF_SECOND;
  }

  dir = compareDist(knownDistFront, knownDistLeft, knownDistRight);

  switch(dir)
  {
    case 0:
      rotLeft();
      delay(ONE_AND_HALF_SECOND);
      break;
    case 1:
      rotRight();
      delay(ONE_AND_HALF_SECOND);
      break;
    case 2:
      moveBackward();
      delay(ONE_AND_HALF_SECOND);
      break;
    default:
      moveForward();
      break;
  }
}

int compareDist(int front, int left, int right)
{
  // left 0, right 1, back 2
  if(right <= RIGHT_LIMIT && left <= LEFT_LIMIT)
  {
    Serial.println("DEFAULT " + String(left) + ", " + String(right));
    return 2;
  }
  else if(left <= LEFT_LIMIT)
  {
    Serial.println("RIGHT" + String(left) + ", " + String(right));
    return 1;
  }
  else if(right <= RIGHT_LIMIT)
  {
    Serial.println("LEFT" + String(left) + ", " + String(right));
    return 0;
  }
  else if(FRONT_LIMIT <= front)
  {
    return 3;
  }
  else if(left < right)
  {
    return 0;
  }
  else if(right < left)
  {
    return 1;
  }
  else
  {
    return 3;
  }
}

//unused, but activates booleans in case of obstacles around
void checkObstacles()
{
  if(knownDistFront <= FRONT_LIMIT)
    obstacleFront = true;
  else
    obstacleFront = false;

  if(knownDistLeft <= LEFT_LIMIT)
    obstacleLeft = true;
  else
    obstacleLeft = false;

  if(knownDistRight <= RIGHT_LIMIT)
    obstacleRight = true;
  else
    obstacleRight = false;
}

void moveForward()
{
  stopped = false;
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
}

void rotLeft()
{
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
}

void rotRight()
{
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void moveBackward()
{
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void stop()
{
  stopped = true;
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

//unused
void wiggle()
{
  /*
  if the robot senses something in front of it wiggle a fixed amount
  left and right to get different measurements from the PING sensor
  if distance stays relatively the same when wiggling, then its a wall,
  if it increases by the length of the metal pickup object or more,
  then it is the object and it should straighten out and pick up the object.
  */
  rotLeft();
  delay(ONE_SECOND);
  rotRight();
  delay(TWO_SECONDS);
  rotLeft();
  delay(ONE_SECOND);
}

void moveArm(int degree)
{
  //65-130
  /*
  turn on the electromagnet first, then lower the arm over the object
  raise the arm until it is overhead, verify this by checking if the PING
  sensor distance has changed, then turn off electromagnet and hold arm
  in place above head.
  */
  ARM_SERVO2.write(degree);
  ARM_SERVO.write(SERVO_OFFSET - degree);
}

void wiggleArm(int degrees)
{
  //corrects for servo offset
  int currentRot1 = SERVO_OFFSET - ARM_SERVO.read();
  int currentRot2 = ARM_SERVO2.read();
  
  for(int i = 0; i < 4; i++)
  {
    delay(FIFTH_SECOND);
    ARM_SERVO2.write(currentRot2 - (degrees));
    ARM_SERVO.write(SERVO_OFFSET - (currentRot1 - degrees) - SERVO_OFFSET_WIGGLE);

    delay(FIFTH_SECOND);
    ARM_SERVO2.write(currentRot2 + (degrees));
    ARM_SERVO.write(SERVO_OFFSET - (currentRot1 + degrees + SERVO_OFFSET_WIGGLE));
  }
}

void setElectromagnetState(bool turnOn)
{
  /*
  turns on the 5v that the electromagnet is hooked up to, separate function
  for better control and possibly unique circumstances
  */
  if(turnOn) {
    digitalWrite(MAGNET, HIGH);
  } else {
    digitalWrite(MAGNET, LOW);
  }
}

long getDistFromPING(int trigger, int recieve)
{
  flashSensor(trigger);
  //  74 / 2 converts microseconds to inches
  dist = (pulseIn(recieve, HIGH) / 74 / 2);

  return dist;
}

void flashSensor(int pin)
{
  //flashes the pins to get more accurate readings
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}
