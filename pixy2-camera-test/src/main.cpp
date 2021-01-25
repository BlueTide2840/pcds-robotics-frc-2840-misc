#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Pixy2.h>
#include <Wire.h>

#define MIN_SERVO_X 50
#define MAX_SERVO_X 160

#define MIN_SERVO_Y 110
#define MAX_SERVO_Y 170

#define FRAME_WIDTH 316
#define FRAME_HEIGHT 208

#define Y_SERVO_PIN 8
#define X_SERVO_PIN 9

Pixy2 pixy;

Servo servoX;
Servo servoY;

//Define Variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

//Specify the links and initial tuning parameters
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 2, 5, 1, P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
// PID myPID(&pidInput, &pidOutput, &pidSetpoint, 2, 5, 1, DIRECT);

uint16_t xPos = 128;
uint16_t yPos = 128;

void setServo(Servo s, int pos, int min, int max, int maxFrame)
{
  s.write(map(pos, 1, maxFrame, min, max));
}

void setX(int pos)
{
  xPos = pos;
  setServo(servoX, pos, MIN_SERVO_X, MAX_SERVO_X, FRAME_WIDTH);
}

void setY(int pos)
{
  setServo(servoY, pos, MIN_SERVO_Y, MAX_SERVO_Y, FRAME_HEIGHT);
}

uint16_t fWidth;
uint16_t fHeight;

double trueMap(double val, double valHigh, double valLow, double newHigh, double newLow)
{
	double midVal = ((valHigh - valLow) / 2) + valLow;
	double newMidVal = ((newHigh - newLow) / 2) + newLow;
	double ratio = (newHigh - newLow) / (valHigh - valLow);
	return (((val - midVal) * ratio) + newMidVal);
}

uint8_t sentData[1];

void requestEvent() {
  Wire.write(sentData[0]);
}

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  servoX.attach(X_SERVO_PIN);
  servoY.attach(Y_SERVO_PIN);

  // We need to initialize the pixy object
  pixy.init();
  fWidth = pixy.frameWidth;
  fHeight = pixy.frameHeight;

  // Use color connected components program for the pan tilt to track
  pixy.changeProg("color_connected_components");

  //turn the PID on
  pidSetpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-158, 158);
  Wire.begin(0x54);
  Wire.onRequest(requestEvent());
}

int panOffset;

int moveDistance = 10;

unsigned long currentTime, previousTime;
void loop()
{
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {

    int x = (158 - pixy.ccc.blocks[0].m_x);
    // Serial.println(x);

    int center = FRAME_WIDTH / 2;

    pidInput = x;
    myPID.Compute();
    Serial.println(pidOutput);

    moveDistance = pidOutput;
    if (moveDistance > 10)
    {
      xPos = x < center ? xPos + moveDistance : xPos - moveDistance;
      setX(xPos);
    }
  }
  else // no object detected, go into reset state
  {
    setX(50);
  }
  sentData[0] = trueMap((158 - pixy.ccc.blocks[0].m_x), 158, -158, 100, 0);
}
