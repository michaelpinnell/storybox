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

#define SCANNER1_SHOOT_PIN              5    // Pin that triggers scanner 1
#define SCANNER2_SHOOT_PIN              6    // Pin that triggers scanner 2

#define VACUUM_PUMP_PIN                 500  // Pin for the vacuum pump that sucks up photos

#define PHOTOCELL_PIN                   A0   // The photocell with a laser being shot at it
#define PHOTOCELL_OBFUSCATION_THRESHOLD 850  // The threshold for laser which triggers scanner

#define FEEDER_ARM_1_SERVO_PIN          9    // The pin for the first feeder arm servo
#define FEEDER_ARM_2_SERVO_PIN          10   // The pin for the second feeder arm servo

#define FEEDER_ARM_SERVO_STEP_DELAY     45   // The delay in milliseconds between each servo step
#define FEEDER_ARM_PICKUP_ANGLE         160  
#define FEEDER_ARM_SCAN_ANGLE           5
#define FEEDER_ARM_DISCARD_ANGLE        95
#define FEEDER_ARM_DISCARD_DELAY        3000

  
//#define VACUUM_PIN                      ?

class ArmServo
{
  public:
    ArmServo(int servoPin)
    /*****************************************************************************/
    {
      _servo.attach(servoPin);
      _servoPin = servoPin;
      _position = _servo.read();
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
        Serial.print("Servo ");
        Serial.print(_servoPin);
        Serial.println(_position);
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
      pinMode(_motorDirectionPin, OUTPUT);     
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
    }

    void turnOn()
    /*****************************************************************************/
    {
      //Complete
    }

    void turnOff()
    /*****************************************************************************/
    {
      //Complete
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
VacuumPump       vacuum(VACUUM_PUMP_PIN);

void setup()
/*****************************************************************************/
{
  //Initialize serial comms for debugging
  Serial.begin(9600);

  //Bring feeder arms to center
  arm1.moveTo(FEEDER_ARM_DISCARD_ANGLE);
  arm2.moveTo(FEEDER_ARM_DISCARD_ANGLE);
}

void loop()
/*****************************************************************************/
{
  //Get next photo ready 
  photoTray.calibrate();

  //Take a photo of the picture
  camera.takePicture();

  //Move the arm to pickup picture
  arm1.moveTo(FEEDER_ARM_PICKUP_ANGLE);
  delay(1000);

  //Move the arm to pickup
  arm1.moveTo(FEEDER_ARM_PICKUP_ANGLE);
  scanner1.takePicture();
  delay(50000);

  arm1.moveTo(FEEDER_ARM_DISCARD_ANGLE);
  vacuum.turnOff();
  delay(FEEDER_ARM_DISCARD_DELAY);
 }

