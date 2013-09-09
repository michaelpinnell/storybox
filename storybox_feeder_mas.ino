//Feeder mechanism
//----------------
//
// This code drives the feeder mechanism consisting of two motors (a PICKUP 
// and FEED motor) that we group together as the FEEDER MOTORS.
// The motors pull a photo off the stack, wait for it to cross a threshold,
// take its picture, and then repeat.
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

#define MOTOR1_STEP_PIN 1    // Motor that first grabs the photo (PICKUP)
#define MOTOR1_DIR_PIN  2      // Motor that first grabs the photo (PICKUP)
#define MOTOR2_STEP_PIN 3  // Motor that moves the photo towards the scanner (FEED)
#define MOTOR2_DIR_PIN  4        // Motor that moves the photo towards the scanner (FEED)
#define PHOTOCELL_PIN   6        // The photocell with a laser being shot at it

// pin 8 and 7 camera
// pin 6 and 5 are photo cell
enum FeederState
{
  FS_ENABLE_FEEDER_MOTORS_AND_WAIT_FOR_NEXT_PHOTO_START_EDGE,
  FS_WAIT_FOR_PHOTO_SECOND_EDGE_TO_PASS_LASER,
  FS_STOP_PICKUP_MOTOR_AND_FEEDER_AND_TAKE_PHOTO
};

FeederState g_feederState = FS_ENABLE_FEEDER_MOTORS_AND_WAIT_FOR_NEXT_PHOTO_START_EDGE;

/*****************************************************************************/
const char* convertFeederStateToString(int currentState)
/*****************************************************************************/
{
  switch(FeederState(currentState))
  {
    case FS_ENABLE_FEEDER_MOTORS_AND_WAIT_FOR_NEXT_PHOTO_START_EDGE: return "Wait for First Edge";
    case FS_WAIT_FOR_PHOTO_SECOND_EDGE_TO_PASS_LASER: return "Wait For Second Edge";
    case FS_STOP_PICKUP_MOTOR_AND_FEEDER_AND_TAKE_PHOTO: return "Stop and Take Photo";
    default: return "Unknown";
  };
}

void stepFeederMotors()
/*****************************************************************************/
{
  digitalWrite(MOTOR2_DIR_PIN, HIGH);
  digitalWrite(MOTOR1_STEP_PIN, HIGH);
  delayMicroseconds(40);          
  digitalWrite(MOTOR1_STEP_PIN, LOW); 
  delayMicroseconds(40);
  digitalWrite(MOTOR2_STEP_PIN, HIGH);
  delayMicroseconds(40);          
  digitalWrite(MOTOR2_STEP_PIN, LOW); 
  delayMicroseconds(40);  
}

int readPhotocellValue()
/*****************************************************************************/
{
  return(analogRead(PHOTOCELL_PIN));
}

bool isLaserBeamObfuscatedByPhoto()
/*****************************************************************************/
{
  return(readPhotocellValue() < 900);
}

void takePicture()
/*****************************************************************************/
{

}

void executeNextState()
/*****************************************************************************/
{
  switch(g_feederState)
  {
    case FS_ENABLE_FEEDER_MOTORS_AND_WAIT_FOR_NEXT_PHOTO_START_EDGE:
      stepFeederMotors();
      if(isLaserBeamObfuscatedByPhoto())
        g_feederState = FS_WAIT_FOR_PHOTO_SECOND_EDGE_TO_PASS_LASER;
      break;

    case FS_WAIT_FOR_PHOTO_SECOND_EDGE_TO_PASS_LASER:
      stepFeederMotors();
      if(!isLaserBeamObfuscatedByPhoto())
        g_feederState = FS_STOP_PICKUP_MOTOR_AND_FEEDER_AND_TAKE_PHOTO;
      break;

    case FS_STOP_PICKUP_MOTOR_AND_FEEDER_AND_TAKE_PHOTO:
      takePicture();
      g_feederState = FS_ENABLE_FEEDER_MOTORS_AND_WAIT_FOR_NEXT_PHOTO_START_EDGE;
      break;      

    default:
      Serial.println("MAJOR PROBLEM!!! UNKNOWN FEEDER STATE!");
  };
}

void setup()
/*****************************************************************************/
{
  Serial.begin(9600);
  // Set button input pin
  pinMode(MOTOR1_DIR_PIN, OUTPUT);     
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN,OUTPUT);
  pinMode(MOTOR2_STEP_PIN,OUTPUT);
  digitalWrite(MOTOR1_STEP_PIN,LOW);
  digitalWrite(MOTOR2_STEP_PIN,LOW);  
}

void loop()
/*****************************************************************************/
{
  FeederState currentState = g_feederState;
  executeNextState();
  if(currentState != g_feederState) //If old != new
  {
    Serial.print("Feeder transitioned from ");
    Serial.print(convertFeederStateToString(currentState));    
    Serial.print(" == TO == >> ");
    Serial.print(convertFeederStateToString(g_feederState));
    Serial.println(".");
  }
}

