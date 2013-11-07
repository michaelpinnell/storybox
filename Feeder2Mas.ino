//Storybox Feeder
//------------------------------------------
//
// This code drives the new feeder mechanism that utilizes vacuum
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

#include <Servo.h>

#define TRAY_MOTOR_STEP_PIN             7    // Tray motor step pin
#define TRAY_MOTOR_DIR_PIN              8    // Tray motor dir pin
#define CAMERA_SHOOT_PIN                5    // Pin that triggers camera switch
#define SCANNER_SHOOT_PIN               6    // Pin that triggers scanner

#define PHOTOCELL_PIN                   A0   // The photocell with a laser being shot at it
#define PHOTOCELL_OBFUSCATION_THRESHOLD 850  // The threshold for laser which triggers scanner

#define FEEDER_ARM_SERVO_PIN             9    // The pin for the feeder arm servo
#define FEEDER_ARM_SERVO_STEP_DELAY      45   // The delay in milliseconds between each servo step
#define FEEDER_ARM_PICKUP_ANGLE          160  
#define FEEDER_ARM_SCAN_ANGLE            5
#define FEEDER_ARM_DISCARD_ANGLE         95
#define FEEDER_ARM_DISCARD_DELAY         3000
  
//#define VACUUM_PIN                      ?

Servo g_feederServo;
int   g_feederServoPosition = 0;

void moveFeederArmServoPosition(int angleInDegrees, int delayBetweenStepsInMilliseconds);
void stepTrayMotor(int direction);
void lowerTrayByOneStep();
void raiseTrayByOneStep();
void calibrateTray();
int  readPhotocellValue();
bool isLaserBeamObfuscatedByPhoto();
void takePicture();
void takeScan();

void setup()
/*****************************************************************************/
{
  //Initialize serial comms for debugging
  Serial.begin(9600);

  //Setup feeder arm servo
  g_feederServo.attach(FEEDER_ARM_SERVO_PIN );
  g_feederServoPosition = g_feederServo.read();

  //Setup camera/scanner pins
  pinMode(CAMERA_SHOOT_PIN, OUTPUT);
  pinMode(SCANNER_SHOOT_PIN, OUTPUT);

  //Setup tray motor pins
  pinMode(TRAY_MOTOR_DIR_PIN, OUTPUT);     
  pinMode(TRAY_MOTOR_STEP_PIN, OUTPUT);
  digitalWrite(TRAY_MOTOR_STEP_PIN,LOW);
  
  //Bring feeder arm to center
  moveFeederArmServoPosition(FEEDER_ARM_DISCARD_ANGLE, FEEDER_ARM_SERVO_STEP_DELAY);
}

void loop()
/*****************************************************************************/
{
  //Get next photo ready 
  calibrateTray();
  takePicture();   

  //Move photo arm to pickup photo
  moveFeederArmServoPosition(FEEDER_ARM_PICKUP_ANGLE, FEEDER_ARM_SERVO_STEP_DELAY);
  delay(1000);
  
  //Move photo arm back to scanner
  moveFeederArmServoPosition(FEEDER_ARM_SCAN_ANGLE, FEEDER_ARM_SERVO_STEP_DELAY);
  takeScan();
  delay(50000); //50 seconds
  
  moveFeederArmServoPosition(FEEDER_ARM_DISCARD_ANGLE, FEEDER_ARM_SERVO_STEP_DELAY);
  //disableVacuum();
  delay(FEEDER_ARM_DISCARD_DELAY);
  //enableVacuum();
 }

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void calibrateTray()
/*****************************************************************************/
{
  if(isLaserBeamObfuscatedByPhoto())
  {
    while(isLaserBeamObfuscatedByPhoto())
      lowerTrayByOneStep();
  }
  else
  {
    while(!isLaserBeamObfuscatedByPhoto())
      raiseTrayByOneStep();
  }
}

bool isLaserBeamObfuscatedByPhoto()
/*****************************************************************************/
{
  int photocellVal = readPhotocellValue(); 
  //Serial.print("photocell = ");
  //Serial.println(photocellVal);
  return(photocellVal < PHOTOCELL_OBFUSCATION_THRESHOLD);
}


void lowerTrayByOneStep()
/*****************************************************************************/
{
  //Serial.println("Lowering tray...");
  stepTrayMotor(-1);
}

void moveFeederArmServoPosition(int angleInDegrees, int delayBetweenStepsInMilliseconds)
/************************************************************/
{
  int increment = g_feederServoPosition < angleInDegrees ? 1 : -1;
  while(true)
  {
    if(g_feederServoPosition == angleInDegrees)
      break;
    g_feederServoPosition += increment;
    Serial.println(g_feederServoPosition);
    g_feederServo.write(g_feederServoPosition);
    delay(delayBetweenStepsInMilliseconds);
  }
}

void raiseTrayByOneStep()
/*****************************************************************************/
{
  //Serial.println("Raising tray...");
  stepTrayMotor(1);
}

int readPhotocellValue()
/*****************************************************************************/
{
  return(analogRead(PHOTOCELL_PIN));
}

void stepTrayMotor(int direction)
/*****************************************************************************/
{
  digitalWrite(TRAY_MOTOR_DIR_PIN, direction > 0 ? LOW : HIGH);
  digitalWrite(TRAY_MOTOR_STEP_PIN, HIGH);
  delayMicroseconds(60);          
  digitalWrite(TRAY_MOTOR_STEP_PIN, LOW); 
  delayMicroseconds(60);
}

void takePicture()
/*****************************************************************************/
{
  digitalWrite(CAMERA_SHOOT_PIN, LOW);
  delay(50);
  digitalWrite(CAMERA_SHOOT_PIN, HIGH);  //SHOOT
  delay(1500);
  digitalWrite(CAMERA_SHOOT_PIN, LOW);
  delay(1);
}

void takeScan()
/*****************************************************************************/
{
  digitalWrite(SCANNER_SHOOT_PIN, LOW);
  delay(50);
  digitalWrite(SCANNER_SHOOT_PIN, HIGH);  //SHOOT
  delay(1500);
  digitalWrite(SCANNER_SHOOT_PIN, LOW);
  delay(1);
}






