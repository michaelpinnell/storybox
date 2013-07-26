enum TrayState
{
  TS_INITIAL,
  TS_CALIBRATE_MOVE_TO_TOP,
  TS_MOVE_TO_BOTTOM,
  TS_IDLE,
  TS_PHOTO_MOVE_TO_TOP,
  TS_TAKE_PHOTO,
  TS_DONE
};

enum LaserState
{
  LS_INITIAL,
  LS_MOVE_TO_FIRST_EDGE,
  LS_WAIT_FOR_TRAY_TO_DESCEND,
  LS_MOVE_TO_SECOND_EDGE,
  LS_READY_TO_SHOOT
};
//Global Variables
//-----------------------------------------------------------------------------
#define UBER_BUTTON_PIN 13                               // number of the button to control the machine
#define RED_PIN 6                                        // Pin for red portion of RGB LED
#define BUTTON_PIN 4                                     // number of the pushbutton pin
#define CAMERA_SHOOT_PIN 2                               // Pin that controls the shoot function
#define GREEN_PIN 10                                      // Pin for green portion of RGB LED
#define BLUE_PIN 11                                      // Pin for blue portion of RGB LED
#define PHOTOCELL_PIN 0                                 // Pin that controls the photocell
#define MOTOR_DIRECTION_PIN 8                       // controls the direction of the motor
#define MOTOR_STEP_PIN 9                            // controls how the motor moves

#define debounce 25                                // ms debounce period to prevent flickering when pressing or releasing the button
#define holdTime 1000                            // ms hold period: how long to wait for press+hold event
#define timeToFinish 45000                                 // how long to wait before moving to the done state

 
int maxDistance=16000;                                   // default distance for the motor to run
long previousTime;                                       // helps to make the comparison to the time to finish
long Timer=millis();
int photoCellVoltage=0;
int sensorValue = 0;                                     // variable to hold the value of the photocell
int photoNum=0;
long clicks = 0;                                     
TrayState currentTrayState = TS_INITIAL;                 // Marks the state the tray is in currently
LaserState currentLaserState = LS_INITIAL;
boolean trayHasStopped=false;                               // tells us if we're in the idle state or not
boolean buttonPressed;
boolean buttonHeld;



// Button variables
int buttonVal = 0; // value read from button
int buttonLast = 0; // buffered value of the button's previous state
long btnDnTime; // time the button was pressed down
long btnUpTime; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered

//Function Prototypes
//-----------------------------------------------------------------------------

//If the red button is press, the camera will take a shot
void checkIfButtonIsPressedAndTakePictureWithCamera();

//Turns the TrayState enumeration into a string that we can send over serial
char* convertTrayStateToString();

//Turns the LaserState enumeration into a string that we can send over serial
char* convertLaserStateToString();

//Returns true if the photo is under the laser
boolean isPhotoUnderLaser();

//If the button on the scan bed is depressed by the tray, this function will return true, otherwise false
boolean isTrayButtonPressed();

//Does an action based on current tray state and then returns the next tray state to transition to
TrayState executeCurrentTrayState();

//Does an action based on current laser state and then returns the next laser state to transition to
LaserState executeCurrentLaserState();

//Moves platform tray down lead screw down to maxDistance
void moveDown(int clicks);

//Moves platform tray up lead screw back up to zero
void moveUp(int clicks);

//Moves the motor one step based on global variable 'moveDirection'
void moveMotorOneStep();

//Sends debug info back over serial
void printSystemInfoToSerial();

//Starts the non blocking timer
void runTimer();

//Change the color of the RGB LED
void setColor();

//Presses the camera's shoot button and takes a picture
void shootCamera();

//Steps the motor one click
void stepMotorOneClick();

//Steps the motor N clicks
void stepMotorNClicks(int n);

//Changes the current motor direction clcokwise
void toggleDirectionClockwise();

//Changes the current motor direction counterclockwise
void toggleDirectionCounterClockwise();

//Check to see if button was pressed or held
void wasUberButtonheld();

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------



void setup() 
{
  Serial.begin(9600);
  // Set button input pin
  pinMode(UBER_BUTTON_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);  //set the button as an input
    pinMode(CAMERA_SHOOT_PIN, OUTPUT);  // set the pin that controls the shoot function

  //Setup motor pins
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);     
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR_DIRECTION_PIN, LOW);
  digitalWrite(MOTOR_STEP_PIN, LOW);
  //Setup LED pins
  pinMode(RED_PIN,OUTPUT);
  pinMode(GREEN_PIN,OUTPUT);
  pinMode(BLUE_PIN,OUTPUT);


}


//=================================================
void loop()
{
  photoCellVoltage= analogRead(PHOTOCELL_PIN);
  clicks++;
  long Timer=millis();
  if(Timer - previousTime > timeToFinish && trayHasStopped==true)
    currentTrayState=TS_DONE;
  wasUberButtonheld();

  //Determine next Laser and Tray states
  LaserState nextLaserState = executeCurrentLaserState(); 
  TrayState nextTrayState = executeCurrentTrayState();

  //If there was a state transition in any of the two state machines, indicate this over serial
  if ((nextLaserState != currentLaserState) || (nextTrayState != currentTrayState))
  {
    //Update current states to next states
    currentLaserState = nextLaserState;
    currentTrayState = nextTrayState; 
    printSystemInfoToSerial();
  }
  
  //Serial.println(sizeof(int));
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//Function Implemented
//-----------------------------------------------------------------------------

/*****************************************************************************/
char* convertLaserStateToString()
/*****************************************************************************/
{
  switch(currentLaserState)
  {
    case LS_INITIAL: return "Initial";
    case LS_MOVE_TO_FIRST_EDGE: return "First Edge";
    case LS_WAIT_FOR_TRAY_TO_DESCEND: return "Wating";
    case LS_MOVE_TO_SECOND_EDGE: return "Second Edge";
    case LS_READY_TO_SHOOT:  return "Ready to shoot";
    default: return "Unknown";
  };
}

/*****************************************************************************/
char* convertTrayStateToString()
/*****************************************************************************/
{
  switch(currentTrayState)
  {
    case TS_INITIAL: return "Initialize";
    case TS_CALIBRATE_MOVE_TO_TOP: return "Calibrate Move To Top";
    case TS_MOVE_TO_BOTTOM: return "Move to Bottom";
    case TS_IDLE: return "Idle";
    case TS_PHOTO_MOVE_TO_TOP: return "Photo Move To Top";
    case TS_TAKE_PHOTO: return "Take Photo";
    case TS_DONE: return "Done";
    default: return "Unknown";
  };
}

/****************************************************************************/
boolean isTrayButtonPressed()
/****************************************************************************/
{
  return (digitalRead(BUTTON_PIN) == HIGH);
}

/****************************************************************************/
boolean isPhotoUnderLaser()
/****************************************************************************/
{
  int photoCellVoltage = analogRead(PHOTOCELL_PIN);
  return (photoCellVoltage < 900 == HIGH);
}

/*****************************************************************************/
LaserState executeCurrentLaserState()
/*****************************************************************************/
{
  switch(currentLaserState)
  {
    case LS_INITIAL:
      return LS_MOVE_TO_FIRST_EDGE;
    case LS_MOVE_TO_FIRST_EDGE:
      if(isPhotoUnderLaser())
        {
          Serial.println("waiting");
          return LS_WAIT_FOR_TRAY_TO_DESCEND;
        }
      else
        return LS_MOVE_TO_FIRST_EDGE;
    case LS_WAIT_FOR_TRAY_TO_DESCEND:
      if(currentTrayState == TS_IDLE)
          return LS_MOVE_TO_SECOND_EDGE;
      else if(currentTrayState != TS_IDLE)
          return LS_WAIT_FOR_TRAY_TO_DESCEND;
    case LS_MOVE_TO_SECOND_EDGE:
      if(!(isPhotoUnderLaser()))
        {
          Serial.println("ready to shoot");
          return LS_READY_TO_SHOOT;
        }
      else
        return LS_MOVE_TO_SECOND_EDGE;
    case LS_READY_TO_SHOOT:
      if(currentTrayState == TS_IDLE)
        return LS_MOVE_TO_FIRST_EDGE;
      else
        return LS_READY_TO_SHOOT;
  };
}

/*****************************************************************************/
TrayState executeCurrentTrayState()
/*****************************************************************************/
{
  switch(currentTrayState)
  {
    case TS_INITIAL:
    setColor(0,255,0);                        //set LED green, we've started our process
    if (buttonPressed == true || buttonHeld == true)
      return TS_CALIBRATE_MOVE_TO_TOP;
    else
      return TS_INITIAL;
    
    case TS_CALIBRATE_MOVE_TO_TOP:
      moveUp(1);
      if(isTrayButtonPressed())
      {
        Serial.println("Button Pressed!");
        clicks = 0;
        return TS_MOVE_TO_BOTTOM;
      }
      else
      return TS_CALIBRATE_MOVE_TO_TOP;

    case TS_MOVE_TO_BOTTOM:
      moveDown(1);
      if(clicks < maxDistance)
        return TS_MOVE_TO_BOTTOM;
      else if (clicks >= maxDistance)
      {
        Serial.println("Awaiting Instuctions");
        
        return TS_IDLE;
      }

    case TS_IDLE:
    if(trayHasStopped==false)
      {
        previousTime=Timer;
        trayHasStopped=true;
        Serial.println(previousTime);
      }
      Timer=millis();
      //Serial.println(Timer);
      if(currentLaserState == LS_READY_TO_SHOOT)
        return TS_PHOTO_MOVE_TO_TOP;
      else
      {
        
        return TS_IDLE;
      }
       

    case TS_PHOTO_MOVE_TO_TOP:
      trayHasStopped=false;
      moveUp(1);
      if(isTrayButtonPressed())
      {
        clicks = 0;
        return TS_TAKE_PHOTO;
      }
      return TS_PHOTO_MOVE_TO_TOP;

    case TS_TAKE_PHOTO:
    Serial.println("Shoot!");
      shootCamera();
      if(buttonPressed==true)
      {
        buttonPressed=false;
        return TS_DONE;
      }
      return TS_MOVE_TO_BOTTOM;
      
    case TS_DONE:
    setColor(0,0,255);
    if(buttonPressed== true || buttonHeld == true)
    {
      clicks=0;
      setColor(0,255,0);
      return TS_MOVE_TO_BOTTOM;
    }
    return TS_DONE;
  
    default:
    setColor(255,0,0);   //red led means things have gone wrong
      //Should never reach here, might be helpful to send an error message over serial
      break;
  };
}



/*****************************************************************************/
void moveDown(int clicks)
/*****************************************************************************/
{
  //counterclock
  toggleDirectionCounterClockwise();
  stepMotorNClicks(clicks);
}

/*****************************************************************************/
void moveUp(int clicks)
/*****************************************************************************/
{
  //clockwise
  toggleDirectionClockwise();
  stepMotorNClicks(clicks);
}

/*****************************************************************************/
void printSystemInfoToSerial()
/*****************************************************************************/
{
  Serial.print("-- Current Tray State = ");
  Serial.print(convertTrayStateToString());
  Serial.print(" -- Current Laser State = ");
  Serial.print(convertLaserStateToString());
  Serial.print(" -- Current clicks = ");
  Serial.print(clicks);
  Serial.println(" -- ");
}

/*****************************************************************************/
void runTimer()
/*****************************************************************************/
{
                              //Non blocking timer to control the LED
  
}
/*****************************************************************************/
void setColor(int red, int green, int blue)
/*****************************************************************************/
{
  analogWrite(RED_PIN,red);
  analogWrite(GREEN_PIN,green);
  analogWrite(BLUE_PIN,blue);
}
/*****************************************************************************/
void shootCamera()
/*****************************************************************************/
{ 
  digitalWrite(CAMERA_SHOOT_PIN,HIGH);  //SHOOT
  //delay(22000); //TODO: Undo this comment and comment next line
  delay(100);
  digitalWrite(CAMERA_SHOOT_PIN,LOW);
  delay(1);
  maxDistance= maxDistance + 500;
}

/*****************************************************************************/
void stepMotorOneClick()
/*****************************************************************************/
{
  digitalWrite(MOTOR_STEP_PIN, HIGH);
  delayMicroseconds(40);          
  digitalWrite(MOTOR_STEP_PIN, LOW); 
  delayMicroseconds(40);
}

/*****************************************************************************/
void stepMotorNClicks(int n)
/*****************************************************************************/
{
  for(int c = 0; c < n; c++)
    stepMotorOneClick();
}

/*****************************************************************************/
void toggleDirectionClockwise()
/*****************************************************************************/
{
  digitalWrite(8, LOW);
}

/*****************************************************************************/
void toggleDirectionCounterClockwise()
/*****************************************************************************/
{
  digitalWrite(8, HIGH);
}

/*****************************************************************************/
void wasUberButtonheld()
/*****************************************************************************/
{
    // Read the state of the button
  buttonVal = digitalRead(UBER_BUTTON_PIN);

  // Test for button pressed and store the down time
  if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnUpTime) > long(debounce))
  {
    btnDnTime = millis();
  }

  // Test for button release and store the up time
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnDnTime) > long(debounce))
  {
    if (ignoreUp == false)
      buttonPressed=true;
    else 
      ignoreUp = false;
    btnUpTime = millis();
  }

  // Test for button held down for longer than the hold time
  if (buttonVal == HIGH && (millis() - btnDnTime) > long(holdTime))
  {
    buttonHeld= true;
    ignoreUp = true;
    btnDnTime = millis();
  }

  buttonLast = buttonVal;
}
