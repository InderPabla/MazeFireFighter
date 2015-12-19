/*
  MazeFireFighter

  Robot traverses the maze to find and put out fires.
  It right wall following algorithm to keep within the
  maze wall. Once it detects a fire, it using fire 
  fighting algorithm to put it out.  

  The circuit:
  * 3 Ultrasonic sensors (2 logic pins each)
  * 2 fire sensors (1 logic pins each)
  * 1 l293d IC (4 logic pins, 2 per wheel)
  * 1 motor for fan (2 digital pins for power)

  December, 18, 2015
  Inderpreet Pabla
*/

//Motors must be on PMW pins (Pulse Width Modulation)
const byte right_motor_pin_1 = 3; //PMW pin
const byte right_motor_pin_2 = 9; //PMW pin
const byte left_motor_pin_1 = 5; //PMW pin
const byte left_motor_pin_2 = 6; //PMW pin

//UltraSonic Sensor Pins
//Left Sensor
const byte trigPinLeft = 2; //Trigger pin
const byte echoPinLeft = 4; //Echo Pin

//RightSensor
const byte trigPinRight = A3; //Trigger pin
const byte echoPinRight = A4; //Echo Pin

//Middle Sensor
const byte trigPinMiddle = A0; //Trigger pin
const byte echoPinMiddle = 8; //Echo Pin

//Power pins for the fan, need 2 digital pins to get desired speed on the motor
const byte fanPin_1 = 10;
const byte fanPin_2 = 7;

//Fire sensor logic pins
const byte firePin_1 = A1; //left fire sensor
const byte firePin_2 = A5; ///right fire sensor

const byte rightTurnPowerShort = 25; //Used when going around a 90 degree corner. Adjustable to match the car (%).
const byte rightTurnPowerLong = 35; //Used when going around a 180 degree corner. Adjustable to match the car (%).

const int quarterTurnTime =750; //Time it takes for car to do a 90 degree turn. Adjustable to match the car (milliseconds).

const int fireBound = 500;
const int sweetSpotLowerBound = 110;
const int sweetSpotUpperBound = 140;
const int leftTightSpotBound = 60;
const int middleWallBound = 100;
const int sidesWallBound = 250;

long millimeterRight, millimeterLeft, millimeterMiddle; //millimeter measurment from ultrasonic sensors

//Fire readings from fire sensor logic pins
int fireValue_1 = 0; //Value of left fire sensor
int fireValue_2 = 0; //Value of right fire sensor

byte turnCounter = 0; //How many short turns have been made using power of rightTurnPowerShort to find the right wall 
boolean fireDetected = false; //true = fire detected, false = not detected

byte fireCounter = 0; //Number of fires that have been put out

boolean debugging = false; //debugging to check for values

/*
 * Function:  setup
 * ----------------
 * Initialize pins and start serial monitoring depending on debugging state.
 */
void setup() 
{
  //initialize pins modes for wheel logic
  pinMode(right_motor_pin_1,OUTPUT); //right wheel
  pinMode(right_motor_pin_2,OUTPUT); //right wheel
  pinMode(left_motor_pin_1,OUTPUT); //left wheel
  pinMode(left_motor_pin_2,OUTPUT); //left wheel

  //initialize pins modes for sensor logic 
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinMiddle, OUTPUT);
  pinMode(echoPinMiddle, INPUT);

  //initialize pins modes for fan power
  pinMode(fanPin_1,OUTPUT); 
  pinMode(fanPin_2,OUTPUT);
  digitalWrite(fanPin_1,LOW); //turn off fan
  digitalWrite(fanPin_2,LOW); //turn off fan
  
  delay(2000); //short delay of 2 seconds to allow user to place the car down

  getDistanceReadings(); //get initial ultrasonic sensor readings 
  getFireReadings(); //get initial fire readings

  if(debugging == true)
  {
    //if debugging is true start serial monitor
    Serial.begin(9600); //using 9600 as baud rate  
    Serial.println("Started");
  }
}

/*
 * Function:  loop
 * ---------------
 * Get sensor readings and perform car action.
 */
void loop() 
{  
  getDistanceReadings(); //get distance readings
  getFireReadings(); //get fire readings
  
  carAction(); //actions based on sensor readings

  if(debugging == true)
  {
    printSensorReadings(); //print sensor readings if debugging is true
  }
}

/*
 * Function:  fightFire
 * --------------------
 * Robot checks makes sure there is a fire and fights the fire. Once the 
 * robot finishs its blowAtFire sweeping method, then the robot re-checks 
 * if the fire has been put out. If the secondary check proves the fire has 
 * been put out then fire counter is update and robot can carry on with
 * traversing the maze, otherwise robot must repeat blowAtFire.
 */
void fightFire()
{
   getDistanceReadings(); //get distance readings

   //while any of the sensors detect fire
   while(fireValue_1<=fireBound || fireValue_2<=fireBound)
   {
     blowAtFire(); //blow at fire using sweeping method

     //if fire has been put out re-check surroundings 
     if(fireValue_1>fireBound|| fireValue_2>fireBound)
     {
        //turn left and check sensors for fire (~45 degrees)
        turnLeftForwardTightVariable(100);
        delay(400);
        turnOff();
        delay(100);
        int fireLeft1 = analogRead(firePin_1); //left turn, left sensor reading
        int fireLeft2 = analogRead(firePin_2); //left turn, right sensor reading

        //turn back to middle
        turnRightForwardTightVariable(100);
        delay(400);
        turnOff();
        delay(100);

        //turn right and check sensors for fire (~45 degrees)
        turnRightForwardTightVariable(100);
        delay(400);
        turnOff();
        delay(100);
        int fireRight1 = analogRead(firePin_1); //right turn, left sensor reading
        int fireRight2 = analogRead(firePin_2); //right turn, right sensor reading 

        //turn back to middle
        turnLeftForwardTightVariable(100);
        delay(400);
        turnOff();
        delay(100);

        //if any of the 4 values show fire then set global fire values to less then fireBound (in this case to 40)
        if(fireRight1<=fireBound || fireLeft1<=fireBound || fireLeft2<=fireBound || fireRight2<=fireBound)
        {
          fireValue_1 = 40;
          fireValue_2 = 40;
        }
     }
   }
    
   getDistanceReadings(); //get distance readings
   fireCounter++; //fire has been extinguished, so update fire counter
   fireDetected = false; //fire detected to false
}

/*
 * Function:  blowAtFire
 * ---------------------
 * Robot tries to stay close to the right wall as it fights
 * the fire. It does a slow sweep turn with the fan on to left and right
 * in order to extinguish the fire.
 */
void blowAtFire()
{
  //move forward a little if the car is between 12 cm and 13 cm to the right wall 
  if(millimeterRight>=sweetSpotLowerBound+10 && millimeterRight<=sweetSpotUpperBound-10)
  { 
    //move forward a little bit
    forwardVariable(100);
    delay(500);
    turnOff();
  }
  //if the car is closer then 12 cm, then turn left for 1/20th of a second and move forward a little 
  else if(millimeterRight<sweetSpotLowerBound+10)
  {
     //turn left a little
     turnLeftForwardTightVariable(100);
     delay(50);
     turnOff();
     delay(50);
     //move forward a little
     forwardVariable(100);
     delay(500);
     turnOff();
  }
  //if the car is further then 13 cm, then turn right for 1/20th of a second and move forward a little 
  else if(millimeterRight>sweetSpotUpperBound-10)
  {
      //turn right a little
     turnRightForwardTightVariable(100);
     delay(50);
     turnOff();
     delay(50);
     //move forward a little
     forwardVariable(100);
     delay(500);
     turnOff();
  } 

   //turn on fan
   digitalWrite(fanPin_1,HIGH);
   digitalWrite(fanPin_2,HIGH);
   delay(3000); //stay in middle for 3 seconds

   //sweep to the left for 1 second (~45 degree turn)
   leftPowerBack(40);
   rightPower(40);
   delay(1000);

   //sweep back to middle
   leftPower(40);
   rightPowerBack(40);
   delay(1000);

   //stay in middle for 1 second
   turnOff();
   delay(1000);

   //sweep to the right for 1 second (~45 degree turn)
   leftPower(50);
   rightPowerBack(50);
   delay(1000);

   //sweep back to middle
   leftPowerBack(50);
   rightPower(50);
   delay(1000);

    //stay in middle for 1 second
   turnOff();
   delay(1000);

   //turn off fan
   digitalWrite(fanPin_1,LOW);
   digitalWrite(fanPin_2,LOW);
   
   delay(1000); //wait 1 second for fan to stop
   
   getDistanceReadings(); //get distance readings
   getFireReadings(); //get fire sensor readings
}

/*
 * Function:  aroundTheCornerFireCheck
 * -----------------------------------
 * Checking for fire when car is going around a corner. The car
 * sweeps left and and right to check for fire. If fire is found 
 * then car goes into fire fighting mode.
 */
void aroundTheCornerFireCheck()
{
  //sweep to the right
  turnRightForwardTightVariable(100);
  delay(250);
  turnOff();
  delay(100);
  int right1  = analogRead(firePin_1); //get right sweep left sensor data
  int right2  = analogRead(firePin_2); //get right sweep right sensor data

  //sweep back to the middle
  turnLeftForwardTightVariable(100);
  delay(250);
  turnOff();
  delay(100);
  int middle1  = analogRead(firePin_1); //get middle sweep left sensor data
  int middle2  = analogRead(firePin_2); //get middle sweep right sensor data

  //sweep to the left
  turnLeftForwardTightVariable(100);
  delay(250);
  turnOff();
  delay(100);
  int left1  = analogRead(firePin_1); //get left sweep left sensor data
  int left2  = analogRead(firePin_2); //get left sweep right sensor data

  //sweep back to the middle
  turnRightForwardTightVariable(100);
  delay(250);
  turnOff();
  delay(100);

  //check if any of the 6 values indicate a fire
  if(right1<=fireBound || right2<=fireBound || middle1<=fireBound || middle2<=fireBound || left1<=fireBound ||left2<=fireBound)
  {
    fireDetected = true; //set fire detected to true
    turnOff(); //turn off car
    fightFire(); //fight fire
  }
}

/*
 * Function:  carAction
 * ------------------------
 * Use wall information to determine next course of action for the car.
 */
void carAction()
{
  byte wallNumber = checkForWalls();
  /*1 left
   *2 right
   *3 left+right
   *4 middle
   *5 left+middle
   *6 right+middle
   *7 left+right+middle
   */
  switch (wallNumber) 
  {
    case 1:turnOff(); //this case is delt with in noWallToTheRight method
      break;
    case 2:wallOnRight(); //keeps to the right
      break;
    case 3:wallOnRight(); //keeps to the right
      break;
    case 4:turnOff(); //this case is delt with in noWallToTheRight method
      break;
    case 5:turnOff(); //this case is delt with in noWallToTheRight method
      break;
    case 6:wallOnMiddleAndRight(); //make a 90 degree left turn
      break;
    case 7:allSides(); //makes a 180 degree turn if there is enough space
      break;
    default:turnOff(); //this case is delt with in noWallToTheRight method
      break;
  }
}

/*
 * Function:  allSides
 * -------------------
 * If there is walls on all sides then the robot first checks if
 * there is enough space to make a turn around. If there is not
 * enough space then the robot backs up to a point were it can.
 * If there is enough space, then the robot turns around and moves 
 * a little closer to the right wall to make sure it can require it. 
 */
void allSides()
{
  getDistanceReadings(); //get distance readings

  //if there is not enough room then back up till there is enough room on the left side
  if(millimeterLeft<=leftTightSpotBound)
  {
    while(millimeterLeft<=leftTightSpotBound)
    {
      backwardVariable(100);
      getDistanceReadings();
    }
  }
  
  turnAround(); //180 degree turn

  //get closer to the right wall
  turnRightForwardTightVariable(100);
  delay(500);
  forwardVariable(100);
  delay(500);
  turnLeftForwardTightVariable(100);
  delay(600);
  turnOff();
  
  turnCounter = 0; //set turn counter to 0
}

/*
 * Function:  wallOnRight
 * ----------------------
 * If there is wall on the right then follow the keep to the right algorithm 
 */
void wallOnRight()
{
    keepToRight(); //keep to right
    turnCounter = 0;//set turn counter to 0
}

/*
 * Function:  wallOnMiddleAndRight
 * -------------------------------
 * Since the robot cannot got forward anymore, it must do a 90 degree turn
 * to the left inorder to make the current middle wall its right wall.
 */
void wallOnMiddleAndRight()
{
    turnLeft(); //turn left 
    turnCounter = 0;//set turn counter to 0
}

/*
 * Function:  keepToRight
 * ----------------------
 * Keep right wall between 12 and 14 centimeters. If the car is in the
 * sweet spot then keep moving forward, otherwise turn left or right
 * to get back in the sweet spot. This will keep the car aligned straight
 * with the right wall.
 * Check sensors for fire, and if turn counter is greater then 2 then do
 * an active sweep check for fire by turning the car left and right. 
 */
void keepToRight()
{
  //if car is in the sweet spot then move forward
  if(millimeterRight>=sweetSpotLowerBound && millimeterRight<=sweetSpotUpperBound)
  { 
    forwardVariable(100);
  }
  //if the car is too close the wall then turn left
  else if(millimeterRight<sweetSpotLowerBound)
  {
     turnLeftForwardTightVariable(100);
  }
  //if the car is too far awau from the wall then turn right
  else if(millimeterRight>sweetSpotUpperBound)
  {
     turnRightForwardTightVariable(100);
  }

  //check for fire values
  if(fireValue_1<=fireBound || fireValue_2<=fireBound)
  {
     fireDetected = true; 
     turnOff();
     fightFire();
  }

  //if turn is >=2 then do an active fire check by sweeping the car left and right
  if(turnCounter>=2)
  {
    aroundTheCornerFireCheck(); //sweep check for fire
  }
  
  turnCounter = 0;//set turn counter to 0
}

/*
 * Function:  checkForWalls
 * ------------------------
 * Use sensor readings to check if there are walls to left, right or middle.
 * return: number which corresponds to were the walls are around the robot
 */
byte checkForWalls()
{
   /*
    * 1 left
    * 2 right
    * 3 left+right
    * 4 middle
    * 5 left+middle
    * 6 right+middle
    * 7 left+right+middle
    */
    
  byte wallNumber = 0;
  
  if(millimeterLeft<=sidesWallBound)
    wallNumber += 1; //wall to the left
  if(millimeterRight<=sidesWallBound)
    wallNumber += 2; //wall to the right
  else
    noWallToTheRight(); //if there is no wall to right then run right wall finding algorithm
  
  
  if(millimeterMiddle<=middleWallBound)
    wallNumber += 4; //wall in in the middle
    
  return wallNumber;
}

/*
 * Function:  noWallToTheRight
 * ---------------------------
 * If there is no wall to the right then the robot can start to 
 * slowly turn towards the right to re-find the right wall. 
 * The first 2 turns at a long arc. If the robot has not found 
 * the right wall using these two long turns then that must mean 
 * the wall is at a 90 degree angle or more. The robot then does 
 * tighter turns to find the right wall. 
 * 
 * If fire sensors detect fire then the robot goes into fire
 * fighting mode.
 */
void noWallToTheRight()
{
  turnCounter++; //update turnCounter

  //if counter >=3 then do tighter right turns
  if(turnCounter>=3)
  {
    leftPower(100);
    rightPower(rightTurnPowerShort);
    delay(1000);
    turnOffWait();
  }
  //if counter <3 then do longer right turns
  else
  {
    leftPower(100);
    rightPower(rightTurnPowerLong);
    delay(1000);
    turnOffWait();
  }

  //if eaither of the sensors detect fire then go into fire fighting mode
  if(fireValue_1<=fireBound || fireValue_2<=fireBound)
  {
     fireDetected = true; //fire detected is true
     turnOff(); //turn off car
     fightFire(); //start fire fighting algorithm
  }  
}

/*
 * Function:  ping
 * ---------------
 * Sends an ultrasonic pulse, then waits to recieve an echo back. 
 * The time is takes to recieve an echo is used to calculate 
 * distance to object depending on the orientation of the 
 * sensors.
 * t: trigger pin of an ultrasonic sensor
 * e: echo pin of an ultrasonic sensor
 * return: distance to object in millimeters
 */
long ping(int t, int e)
{
  digitalWrite(t, LOW); //low flush to clear sensor
  delayMicroseconds(2); //wait 2 microseconds for the low flush to kick in
  digitalWrite(t, HIGH); //turn trigger pin to high 
  delayMicroseconds(10); //wait 10 seconds (this will give a reading of ATLEAST 14.5 millimeters (10*2.9/2 = 14.5 mm)
  digitalWrite(t, LOW); //turn trigger to low to stop pulse and wait for echo
  long duration = pulseIn(e, HIGH); //wait for echo pin to recieve the pulse
  return microsecondsToMillimeters(duration); //use microsecond time to calculate distance in millimeters
}

/*
 * Function:  microsecondsToMillimeters
 * ------------------------------------
 * Converts microseconds to distance traveled by sound. 
 * microsecond: time it took to get echo back from sensor
 * return: distance travelled by sound in millimeters
 */
long microsecondsToMillimeters(long microseconds)
{
  //Time it takes sounds to travel 1 mm = (1000 microseconds) / (345 m/seconds) = 2.9 microseconds
  //Distance = (microseconds) / (Time it takes sounds to travel 1 mm) / 2
  return microseconds / 2.9 / 2.0;
}

/*
 * Function:  turnOff
 * ------------------
 * Makes robot turn off.
 */
void turnOff()
{
  digitalWrite(right_motor_pin_1,LOW); 
  digitalWrite(right_motor_pin_2,LOW);
  digitalWrite(left_motor_pin_1,LOW); 
  digitalWrite(left_motor_pin_2,LOW); 
}

/*
 * Function:  turnOffWait
 * ---------------------_
 * Makes robot turn off for half a second. 
 * The reason this method exists is to reduce unnecessary code. 
 */
void turnOffWait()
{
  turnOff();
  delay(500);
}

/*
 * Function:  turnAround
 * ---------------------
 * Makes robot turn off for half a second. 
 * Then it turns left for 2 times quarter turn time.
 * Finally robot turns off for half a second.
 */
void turnAround()
{
   turnOffWait(); 
   turnLeftForwardTight();
   delay(quarterTurnTime*2);
   turnOffWait();
}

/*
 * Function:  turLeft
 * ------------------
 * Makes robot turn off for half a second. 
 * Then it turns left for quarter turn time.
 * Finally robot turns off for half a second.
 */
void turnLeft()
{
   turnOffWait();
   turnLeftForwardTight();
   delay(quarterTurnTime);
   turnOffWait();
}

/*
 * Function:  turRight
 * ------------------
 * Makes robot turn off for half a second. 
 * Then it turns right for quarter turn time.
 * Finally robot turns off for half a second.
 */
void turnRight()
{
   turnOffWait();
   turnRightForwardTight();
   delay(quarterTurnTime);
   turnOffWait();
}

/*
 * Function:  turnLeftForwardTight
 * --------------------------------
 * Makes robot turn left on its center of mass at full speed.
 */
void turnLeftForwardTight()
{
  digitalWrite(right_motor_pin_1,HIGH);
  digitalWrite(right_motor_pin_2,LOW);
  digitalWrite(left_motor_pin_1,LOW);
  digitalWrite(left_motor_pin_2,HIGH);
}

/*
 * Function:  turnRightForwardTight
 * --------------------------------
 * Makes robot turn right on its center of mass at full speed.
 */
void turnRightForwardTight()
{
  digitalWrite(right_motor_pin_1,LOW);
  digitalWrite(right_motor_pin_2,HIGH);
  digitalWrite(left_motor_pin_1,HIGH);
  digitalWrite(left_motor_pin_2,LOW);
}

/*
 * Function:  turnRightForwardTightVariable
 * ----------------------------------------
 * Makes robot turn right on its center of mass with desired speed.
 * amount: % value of speed of robot
 */
void turnRightForwardTightVariable(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(right_motor_pin_1,0);
  analogWrite(right_motor_pin_2,calibratedSpeed);
  analogWrite(left_motor_pin_1,calibratedSpeed);
  analogWrite(left_motor_pin_2,0);
}

/*
 * Function:  turnLeftForwardTightVariable
 * ---------------------------------------
 * Makes robot turn left on its center of mass with desired speed.
 * amount: % value of speed of robot
 */
void turnLeftForwardTightVariable(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(right_motor_pin_1,calibratedSpeed);
  analogWrite(right_motor_pin_2,0);
  analogWrite(left_motor_pin_1,0);
  analogWrite(left_motor_pin_2,calibratedSpeed);
}

/*
 * Function:  leftPower
 * --------------------
 * Makes left wheel move forward with desired speed.
 * amount: % value of speed of robot
 */
void leftPower(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(left_motor_pin_1,calibratedSpeed);
  analogWrite(left_motor_pin_2,0);
}

/*
 * Function:  rightPower
 * ---------------------
 * Makes right wheel move forward with desired speed.
 * amount: % value of speed of robot
 */
void rightPower(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(right_motor_pin_1,calibratedSpeed);
  analogWrite(right_motor_pin_2,0);
}

/*
 * Function:  leftPowerBack
 * ------------------------
 * Makes left wheel move backwards with desired speed.
 * amount: % value of speed of robot
 */
void leftPowerBack(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(left_motor_pin_1,0);
  analogWrite(left_motor_pin_2,calibratedSpeed);
}

/*
 * Function:  rightPowerBack
 * -------------------------
 * Makes right wheel move backwards with desired speed.
 * amount: % value of speed of robot
 */
void rightPowerBack(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(right_motor_pin_1,0);
  analogWrite(right_motor_pin_2,calibratedSpeed);
}

/*
 * Function:  forwardVariable
 * --------------------------
 * Makes robot move forward with desired speed.
 * amount: % value of speed of robot
 */
void forwardVariable(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(right_motor_pin_1,calibratedSpeed);
  analogWrite(right_motor_pin_2,0);
  analogWrite(left_motor_pin_1,calibratedSpeed);
  analogWrite(left_motor_pin_2,0);
}

/*
 * Function:  backwardVariable
 * ---------------------------
 * Makes robot move backwards with desired speed.
 * amount: % value of speed of robot
 */
void backwardVariable(float amount)
{
  int calibratedSpeed = (amount/100.0)*255.0; //calibrate speed out of 255
  //analog write the calibrated speed
  analogWrite(right_motor_pin_1,0);
  analogWrite(right_motor_pin_2,calibratedSpeed);
  analogWrite(left_motor_pin_1,0);
  analogWrite(left_motor_pin_2,calibratedSpeed);
}

/*
 * Function:  getDistanceReadings
 * ------------------------------
 * Gets distance readings on each ultrasonic sensors.
 */
void getDistanceReadings()
{
  millimeterMiddle = ping(trigPinMiddle,echoPinMiddle); //get distance readings on middle sensor.
  millimeterRight = ping(trigPinRight,echoPinRight); //get distance readings on right sensor.
  millimeterLeft  = ping(trigPinLeft,echoPinLeft); //get distance readings on left sensor.
}

/*
 * Function:  getFireReadings
 * --------------------------
 * Gets analog readings from left and right fire sensor logic pins.
 */
void getFireReadings()
{
  fireValue_1  = analogRead(firePin_1); //left fire sensor
  fireValue_2  = analogRead(firePin_2); //right fire sensor
}

/*
 * Function:  printSensorReadings
 * ------------------------------
 * Prints information from various sensors.
 * Prints information about left, right and middle ultrasonic sensors. 
 * Prints information about fire values of left and right fire sensors.
 */
void printSensorReadings()
{
  Serial.print("Distances: ");
  Serial.print(millimeterLeft);
  Serial.print(" ");
  Serial.print(millimeterMiddle);
  Serial.print(" ");
  Serial.print(millimeterRight);
  Serial.print(" - Fire: ");
  Serial.print(fireValue_1);
  Serial.print(" ");
  Serial.print(fireValue_2);
  Serial.print("\n");
}
