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

#define CAMERA_SHOOT_PIN                4    // Pin that triggers camera switch

#define SCANNER1_SHOOT_PIN              5    // Pin that triggers scanner 1
#define SCANNER2_SHOOT_PIN              6    // Pin that triggers scanner 2

#define VACUUM_PUMP1_PIN                3    // Pin for the vacuum pump 1 that sucks up photos
#define VACUUM_PUMP2_PIN                11   // Pin for the vacuum pump 2 that sucks up photos

#define PHOTOCELL_PIN                   A0   // The photocell with a laser being shot at it
#define PHOTOCELL_OBFUSCATION_THRESHOLD 850  // The threshold for laser which triggers scanner

#define FEEDER_ARM_1_SERVO_PIN          9    // The pin for the first feeder arm servo
#define FEEDER_ARM_2_SERVO_PIN          10   // The pin for the second feeder arm servo

#define FEEDER_ARM_SERVO_STEP_DELAY     45   // The delay in milliseconds between each servo step
#define FEEDER_ARM_PICKUP_ANGLE         160  
#define FEEDER_ARM_SCAN_ANGLE           5
#define FEEDER_ARM_DISCARD_ANGLE        95
#define FEEDER_ARM_DISCARD_DELAY        3000


class ArmServo
{
  public:
    ArmServo(int servoPin)
    /*****************************************************************************/
    {
      _servoPin = servoPin; 
    }
    
    void init()
    {
      _servo.attach(_servoPin);
    }
    
    int getPosition() const 
    /*****************************************************************************/
    {
      return _position;
    }

    void moveTo(int angleInDegrees, int delayBetweenStepsInMilliseconds = FEEDER_ARM_SERVO_STEP_DELAY)
    /*****************************************************************************/
    {
      int increment = _position < angleInDegrees ? 1 : -1;
      while(true)
      {
        if(_position == angleInDegrees)
          break;
        _position += increment;
        //Serial.print("Servo ");
        //Serial.print(_servoPin);
        //Serial.println(_position);
        _servo.write(_position);
        delay(delayBetweenStepsInMilliseconds);
      }
    }

  private:
    Servo _servo;
    int   _position; //Degrees 0 <= position <= 180
    int   _servoPin;
};

class Tray
{
  public:
    Tray(int trayMotorStepPin, int trayMotorDirectionPin)
    /*****************************************************************************/
    {
      _motorStepPin = trayMotorStepPin;
      _motorDirectionPin = trayMotorDirectionPin;
      pinMode(_motorStepPin, OUTPUT);     
      pinMode(_motorDirectionPin, OUTPUT);
      digitalWrite(_motorStepPin,LOW);
    }

    void calibrate()
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

  private:
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
      digitalWrite(_motorDirectionPin, direction > 0 ? LOW : HIGH);
      digitalWrite(_motorStepPin, HIGH);
      delayMicroseconds(60);          
      digitalWrite(_motorStepPin, LOW); 
      delayMicroseconds(60);
    }

  private:
    int _motorStepPin;
    int _motorDirectionPin;
};

class ImageInputDevice //Scanner or Camera
{
  public:
    ImageInputDevice(int shootPin)
    /*****************************************************************************/
    {
      _shootPin = shootPin;
      pinMode(shootPin, OUTPUT);
    }

    void takePicture()
    /*****************************************************************************/
    {
      digitalWrite(_shootPin, LOW);
      delay(50);
      digitalWrite(_shootPin, HIGH);  //SHOOT
      delay(1500);
      digitalWrite(_shootPin, LOW);
      delay(1);
    }
    
  private:
    int _shootPin;
};

class VacuumPump
{
  public:
    VacuumPump(int vacuumPin)
    /*****************************************************************************/
    {
      _vacuumPin = vacuumPin;
      pinMode(vacuumPin, OUTPUT);
    }

    void turnOn()
    /*****************************************************************************/
    {
      analogWrite(_vacuumPin, 255);
    }

    void turnOff()
    /*****************************************************************************/
    {
      analogWrite(_vacuumPin, 0);
    }
  
  private:
    int _vacuumPin;
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

ArmServo         arm1(FEEDER_ARM_1_SERVO_PIN);
ArmServo         arm2(FEEDER_ARM_2_SERVO_PIN);
Tray             photoTray(TRAY_MOTOR_STEP_PIN, TRAY_MOTOR_DIR_PIN);
ImageInputDevice camera(CAMERA_SHOOT_PIN);
ImageInputDevice scanner1(SCANNER1_SHOOT_PIN);
ImageInputDevice scanner2(SCANNER2_SHOOT_PIN);
VacuumPump       vacuum1(VACUUM_PUMP1_PIN);
VacuumPump       vacuum2(VACUUM_PUMP2_PIN);

void setup()
/*****************************************************************************/
{
  //Initialize serial comms for debugging
  Serial.begin(9600);
  
  //Initialize arms
  arm1.init();
  arm2.init();

  //Bring feeder arms to center
  arm1.moveTo(FEEDER_ARM_DISCARD_ANGLE);
  arm2.moveTo(FEEDER_ARM_DISCARD_ANGLE);
  
  vacuum1.turnOff();
  vacuum2.turnOff();
}

void loop()
/*****************************************************************************/
{
  //Turn on the vacuum
  Serial.println("Turn on vacuum 1");
  vacuum1.turnOn();
  
  //Get next photo ready 
  Serial.println("Adjust photo tray (arm 1 is next receiver).");
  photoTray.calibrate();

  //Take a photo of the picture
  Serial.println("Take picture of back of photo (for arm1).");
  camera.takePicture();

  //Move the arm to pickup picture
  Serial.println("Moving arm 1 to pickup photo.");  
  arm1.moveTo(FEEDER_ARM_PICKUP_ANGLE);
  delay(1000);

  //Move the arm to the scanner and trigger the scanner
  Serial.println("Moving arm 1 to scanning position.");
  arm1.moveTo(FEEDER_ARM_SCAN_ANGLE);
  Serial.println("Starting scan on scanner 1.");
  scanner1.takePicture();
  
  //While we're scanning on photo scanner 1, let's take a picture of next photo and get arm 2 ready
  Serial.println("Adjust photo tray (arm 2 is next receiver).");
  photoTray.calibrate();
  Serial.println("Take picture of back of photo (for arm2).");
  camera.takePicture();
  Serial.println("Turn on vacuum 2.");
  vacuum2.turnOn();
  Serial.println("Moving arm 2 to pickup photo.");
  arm2.moveTo(FEEDER_ARM_PICKUP_ANGLE);
  Serial.println("Wait for arm 2 to suckup photo.");
  delay(1000);
  Serial.println("Moving arm 2 to scanning position.");
  arm2.moveTo(FEEDER_ARM_SCAN_ANGLE);
  Serial.println("Starting scan on scanner 2.");
  scanner2.takePicture();
  
  //Wait for scanner 1 to finish
  Serial.println("Waiting out remaining time for scan 1");
  delay(10000);

  //Move arm1 to the discard position, disable the vacuum
  Serial.println("Assuming scan on scanner 1 complete, moving arm 1 to discard position.");
  arm1.moveTo(FEEDER_ARM_DISCARD_ANGLE);
  
  Serial.println("Turn off vacuum 1.");
  vacuum1.turnOff();
  
  Serial.println("Waiting a sec for photo to fall off arm 1.");
  delay(FEEDER_ARM_DISCARD_DELAY);  
  
  //Wait for scanner 2 to finish
  Serial.println("Waiting out remaining time for scan 2");
  delay(10000);
  
  //Move arm2 to the discard position, disable the vacuum
  Serial.println("Assuming scan on scanner 2 complete, moving arm 2 to discard position.");
  arm2.moveTo(FEEDER_ARM_DISCARD_ANGLE);
  Serial.println("Turn off vacuum 2.");
  vacuum2.turnOff();
  Serial.println("Waiting a sec for photo to fall off arm 2.");
  delay(FEEDER_ARM_DISCARD_DELAY);   
 }
