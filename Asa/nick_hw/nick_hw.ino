// /*******************************************************************/
/*******************************************************************
 * Created January 2024 by Nick Livingston
 * Edited April 2024 by Nick Livingston
 * 
 * This is baseline code for getting a TinyDuino based two-wheeled
 * vehicle to drive using proportional drive.  Proportional drive
 * in this context means, the left and right motors (servo or DC)
 * are controlled through a drive() function that specifies how
 * much of the maximum possible power should be used to drive
 * each motor, and for how long.  The left_proportion and
 * right_proportion parameters for the drive() function may take
 * values between -1 and 1, where -1 means that motor should be
 * driven at full power "backwards", and +1 means the motor should
 * be driven at full power "forwards".

// drive(forward)
left_proportion = .5
right_proportion = .5






 *
 * This code has been tested and verified for the TinyScreen+ and
 * RobotZero controllers, and works with both DC and servo motors.
 * The constant integer, "MOTOR_TYPE" is used to determine which
 * motor is used in the rest of the code.  "DC_MOTOR" means that
 * a direct current motor will be used, either in the DC motor port
 * on the RobotZero, or in a motor shield for the TinyScreen+.
 * "TS_SERVO" stands for "TinyScreen" servo, and will indicate that
 * a ServoDriver object should be created and initialized with the
 * value NO_R_REMOVED, which comes from the ServoDriver library, and
 * resolves to the value 0.  "RZ_SERVO" stands for "RobotZero" servo
 * and indicates that a ServoDriver object should be created and
 * initialized with the value 15, which is the argument needed for
 * the ServoDriver constructor if a servo is being used with the
 * RobotZero controller.
 * Thus, to use this code, you must know which controller you are
 * using if you are going to drive using servos, or you must set
 * MOTOR_TYPE to DC_MOTOR if you are using a DC motor.  This code
 * can serve as a starting point for more sophisticated robotics
 * projects.  Add code for your sensors and controlling logic, and
 * your two wheeled robot should be on its way!
 *******************************************************************/
/*******************************************************************/
#include <Wire.h>
#include <ServoDriver.h>
#include <MotorDriver.h>
#include "VL53L0X.h"    // For interfacing with the Time-of-Flight Distance sensor
#include "ColorWireling.h"
#include <Wireling.h>   // For interfacing with Wirelings

//#include <string.h>


#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

/*** Motor/Servo Variables and Constants ***/

//Meta Motor
#define DC_MOTOR 1
#define TS_SERVO NO_R_REMOVED
#define RZ_SERVO 15
const int MOTOR_TYPE = TS_SERVO;

const String sLEFT    = "LEFT";
const String sRIGHT   = "RIGHT";
const String sREVERSE =  "REVERSE";
const String sFORWARD = "FORWARD";

//Servo Motor
const int SERVO_HALT = 1500;
const int MAX_SERVO_OFFSET = 500;
const int LEFT_PORT = 1;
const int RIGHT_PORT = 2;

//DC Motor
int maxPWM = 10000;
int steps = 300;
int stepSize = maxPWM / steps;

//If you are not using a DC motor, the value of MOTOR_TYPE
//should be set to "TS_SERVO" if you are using the TinyScreen+ processor
//or "RZ_SERVO" if you are using the RobotZero.  Each of these constants
//has the value needed to pass to the ServoDriver constructor in order
//to properly initialize the servo object for the corresponding controller.
// #if MOTOR_TYPE != TS_Servo
ServoDriver servo(MOTOR_TYPE);// Value passed is the address- RobotZero is always address 15
// #endif 

/*** TOF Sensor Variables and Constants ***/
///VL53L0X leftTofSensor; // Name of sensor 
///const int TOF_LS_PORT = 3;  // Port # of sensor (Found on Wireling Adapter Board)

//VL53L0X rightTofSensor; // Name of sensor 
//const int TOF_RS_PORT = 0;  // Port # of sensor (Found on Wireling Adapter Board)

//const int averageTofCount = 1;
//int averageTof[averageTofCount];
//int averageTofPos = 0;

// global variables to store all current sensor values accessible to all functions and updated by the "read_sensors" function
//The unsigned long type, when used, means we are creating of variable (of type unsigned long, which means) that can store a very large number. Unsigned means it won't store negative numbers (so more space in the computer to store even larger positive number - that's a side note, don't worry too much about it :))
unsigned long right_ir_value, left_ir_value;

// threshold values
unsigned long avoid_threshold = 100;	   // the absolute difference between IR readings has to be above this for the avoid action

/*** COLOR Sensor Variables and Constants ***/
const int LDR_LS_PORT = 2;
ColorWireling leftColorSensor(LDR_LS_PORT);
const int LDR_RS_PORT = 1;
ColorWireling rightColorSensor(LDR_RS_PORT);
const int LDR_CS_PORT = 3;
ColorWireling centerColorSensor(LDR_CS_PORT);
const int LDR_DS_PORT = 0;
ColorWireling debugColorSensor(LDR_DS_PORT);
uint16_t photo_threshold = 400;	   // the absolute difference between photo sensor readings has to be above this for seek light/dark actions
uint16_t right_photo_value, left_photo_value;

//See the notes about functions that I put (currently at line 213, but that might change.).
//The notes precede the function called get_dist_mm
void setup() 
{
  delay(200);              //TOF Sensor Startup time
  SerialMonitorInterface.begin(115200);
  driveInit();
  SerialMonitorInterface.println('🔥');

  //THIS IS SETUP FOR THE WIRELING SENSORS
  //IF YOU JUST WANT TO SEE THE SERVOS RUN, LEAVE THIS OUT.
   Wire.begin();            // Begin I2C communication

  //Enable power & select port
  Wireling.begin(); 

  //RGB Sensor setup
  leftColorSensor.init();
  centerColorSensor.init();
  rightColorSensor.init();
  // leftColorSensor.LEDoff();
  // rightColorSensor.LEDoff();
}
/*******************************************************************/
void loop() 
{
  float test_speed = 0.8;//BTX 0 and 1
  String test_direction = sFORWARD; //Use one of the constants (e.g. sFORWARD)
  float test_duration = 0.5;//specified in seconds
  
  
  // translate(test_speed, test_direction, test_duration);
  // halt(0.5);
  // test_direction = sLEFT;
  // rotate(test_speed, test_direction, test_duration);
  // halt(0.5);
  // test_direction = sRIGHT;
  // turn(test_speed, test_direction, test_duration);
  // halt(0.5);
  // translate(0.3, sFORWARD, 0.5);
  // for(float v = 0.10; v < 0.80; v+=0.1)
  // {
  //   drive(1.0, 1.0, 0.5);
  //   drive(v, v, 0.5);
  // }
  


  read_sensors(); //read all sensors and set global variables of their readouts
   
  lineLogic();
   
  
}
/*******************************************************************/
void read_sensors()
{
  // //Variables local to this function.  These get reused for each of the color sensors, allowing the hue, saturation and color value to be
  // //printed out for each sensor.  Once the function exists, the variables are destroyed.
  // float hue, saturation, color_value;

  //get distance in millimeters.  Keep in mind, these sensors have low accuracy under 2mm, and as they approach 2 meters.
  //This latter issue should not come into play for this design, but you will have to be mindful of how and where you mount the IR sensors
  //so as to not get them too close to the driving surface. 
 
  //USE OF TOF SENSORS OBSOLETE!
  // left_ir_value  = get_dist_mm(leftTofSensor, TOF_LS_PORT);
  // right_ir_value = get_dist_mm(rightTofSensor, TOF_RS_PORT);
 
  Wireling.selectPort(LDR_LS_PORT);
  leftColorSensor.getColorData();
  

  Wireling.selectPort(LDR_CS_PORT);
  centerColorSensor.getColorData();
  


  Wireling.selectPort(LDR_RS_PORT);
  rightColorSensor.getColorData();
  
  //Wireling.selectPort(LDR_RS_PORT);
  //rightColorSensor.getColorData();

  //ColorSensor functions:
  //getH() gets the hue of the perceived color in degrees.
  //getS() gets the satuaration level.
  //getV() gets the value of the color.
  //There are also getR(), getG(), and getB() functions that will get you red, green, and blue channels.
  //All of these can be seen in ColorWireling.cpp
  //left_photo_value  = leftColorSensor.getH();//.getLux(); 
  //right_photo_value = rightColorSensor.getH();//getLux();

  //This prints "hue = (the actual hue value)"; it does the same for saturatio, value (i.e. color value), left IR value, right ir value. It prints that on the serialMonitorInterface. You can find this on the top right (the four dots with like a microscrope). This will print the values every period of time (usually a. few milliseconds).
  //Worth noting the difference between print and println. Print will print whatever is in the parenthesis and quotation marks without returning to the line. So the next thing that is going to be printed will be on the same line.
  //Println, on the other hand, will print whatever is in the parentehsis and quotation marks, then return to the line. So the next thing that will be printed will be on a new line,
  
  // SerialMonitorInterface.print("left_photo_value = ");
  // SerialMonitorInterface.println(left_photo_value, DEC);
  // SerialMonitorInterface.print("right_photo_value = ");
  // SerialMonitorInterface.println(right_photo_value, DEC);
  // SerialMonitorInterface.print("LEFT_TOF = ");
  // SerialMonitorInterface.println(left_ir_value);
  // SerialMonitorInterface.print("RIGHT_TOF = ");
  // SerialMonitorInterface.println(right_ir_value);
  SerialMonitorInterface.print('🧮');
	// read the bumpers
	// front_bump_left_value = digital(FRONT_BUMP_LEFT_PIN);   // read the bumper at FRONT_BUMP_LEFT_PIN
	// front_bump_center_value = digital(FRONT_BUMP_CENTER_PIN); // read the bumper at FRONT_BUMP_CENTER_PIN
	// front_bump_right_value = digital(FRONT_BUMP_RIGHT_PIN);  // read the bumper at FRONT_BUMP_RIGHT_PIN
	// back_bump_left_value = digital(BACK_BUMP_LEFT_PIN);	// read the bumper at BACK_BUMP_LEFT_PIN
	// back_bump_center_value = digital(BACK_BUMP_CENTER_PIN);  // read the bumper at BACK_BUMP_CENTER_PIN
	// back_bump_right_value = digital(BACK_BUMP_RIGHT_PIN);	// read the bumper at BACK_BUMP_RIGHT_PIN	
}
/*******************************************************************/
void sensorCalibration()
{
  SerialMonitorInterface.print('🧮');
}
/*******************************************************************/

void lineLogic()
{
  bool rightIsLine, centerIsLine, leftIsLine;
  bool sensorArray[3] = {};
  float speed;
  float duration;
  String direction;

  float hue = 0.0;
  float saturation = 0.0;
  float color_value = 0.0;

  const int NUM_SAMPLES = 10;
  const int NUM_SENSORS = 3;
  float hueArray[] = {};
  float saturationArray[] = {};
  float colorValueArray[] = {};

  float hueMean[NUM_SENSORS], saturationMean[NUM_SENSORS], colorValueMean[NUM_SENSORS], hueMode[NUM_SENSORS], saturationMode[NUM_SENSORS], colorValueMode[NUM_SENSORS];
   
  for(int n = 0; n < NUM_SENSORS; n++)
  {
    for(int i = 0; i < 10; i++)
    {
      hueArray[i] = leftColorSensor.getH();
      saturationArray[i] = leftColorSensor.getS();
      colorValueArray[i] = leftColorSensor.getV();

      SerialMonitorInterface.print("hueArray[");
      SerialMonitorInterface.print(i);
      SerialMonitorInterface.print("]: ");
      SerialMonitorInterface.println(hueArray[i]);
    }
      
    hueMean[n] = fmean(hueArray,10);
    saturationMean[n] = fmean(saturationArray,sizeof(saturationArray));
    colorValueMean[n] = fmean(colorValueArray,sizeof(colorValueArray));
    hueMode[n] = fmode(hueArray,sizeof(hueArray));
    saturationMode[n] = fmode(saturationArray,sizeof(saturationArray));
    colorValueMode[n] = fmode(colorValueArray,sizeof(colorValueArray));

    //leftIsLine = leftColorSensor.isLine();
    SerialMonitorInterface.print("SENSOR ");
    SerialMonitorInterface.print(n);
    SerialMonitorInterface.println(": ");
    SerialMonitorInterface.print("hue mean =");
    SerialMonitorInterface.println(hueMean[n]);
    SerialMonitorInterface.print("saturation mean =");
    SerialMonitorInterface.println(saturationMean[n]);
    SerialMonitorInterface.print("colorvalue mean =");
    SerialMonitorInterface.println(colorValueMean[n]);
    /////////**/////
    SerialMonitorInterface.print("hue mode =");
    SerialMonitorInterface.println(hueMode[n]);
    SerialMonitorInterface.print("saturation mode =");
    SerialMonitorInterface.println(saturationMode[n]);
    SerialMonitorInterface.print("colorvalue mode =");
    SerialMonitorInterface.println(colorValueMode[n]);
  }
  //
  //
  // SerialMonitorInterface.println(hue);
  //SerialMonitorInterface.println("LEFT");
  // SerialMonitorInterface.print("hue = ");
  //SerialMonitorInterface.print("saturation = ");
  //SerialMonitorInterface.println(saturation);
  // SerialMonitorInterface.print("value = ");
  // SerialMonitorInterface.println(color_value);

  hue = centerColorSensor.getH();
  saturation = centerColorSensor.getS();
  color_value = centerColorSensor.getV();
  
  centerIsLine = centerColorSensor.isLine();

  //SerialMonitorInterface.println("CENTER");
  //SerialMonitorInterface.print("hue = ");
  //SerialMonitorInterface.println(hue);
  //SerialMonitorInterface.print("saturation = ");
  //SerialMonitorInterface.println(saturation);
  //SerialMonitorInterface.print("value = ");
  //SerialMonitorInterface.println(color_value);

  hue = rightColorSensor.getH();
  saturation = rightColorSensor.getS();
  color_value = rightColorSensor.getV();

  rightIsLine = rightColorSensor.isLine();

  //SerialMonitorInterface.println("RIGHT");
  // SerialMonitorInterface.print("hue = ");
  // SerialMonitorInterface.println(hue);
  //SerialMonitorInterface.print("saturation = ");
  //SerialMonitorInterface.println(saturation);
  // SerialMonitorInterface.print("value = ");
  // SerialMonitorInterface.println(color_value);

  //The "isLine()" method within the ColorWireling class returns, "true" if the saturation level
  //seen by the sensor calling the method is above a threshold defined within the isLine() method itself.
  //This is all happening after the call within "loop()" to "readSensors()", however, all of the work done
  //in "readSensors()" is thrown out after that function exists.
  
  //SerialMonitorInterface.print("right =");
  //SerialMonitorInterface.println(rightIsLine);
  //SerialMonitorInterface.print("center =");
  //SerialMonitorInterface.println(centerIsLine);
  //SerialMonitorInterface.print("left =");
  //SerialMonitorInterface.println(leftIsLine);

  sensorArray[0] = rightIsLine;
  sensorArray[1] = centerIsLine;
  sensorArray[2] = leftIsLine;
  

  /*

  if (sensorArray[0] == 0 && sensorArray[1] == 1 && sensorArray[2] == 0)
  {
	  speed = 0.5;//guess? Maybe something else?
	  duration = 0.5; //guess? Maybe something else?
    direction = sFORWARD;
	  translate(speed,sFORWARD,duration);
    SerialMonitorInterface.print("forward: 010");
  }
  if (sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[2] == 1)
  {
	  
	  arc(0.4,sLEFT,1);
    SerialMonitorInterface.print("left arc: 111");
  }
  if (sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[2] == 0)
  {
	  
	  rotate(0.1,sLEFT,1);
    SerialMonitorInterface.print("left: 110");
  }
  if (sensorArray[0] == 0 && sensorArray[1] == 1 && sensorArray[2] == 1)
  {
	  ;
	  rotate(0.1,sRIGHT,1);
    SerialMonitorInterface.print("right: 011");
  }
  
  if (sensorArray[0] == 1 && sensorArray[1] == 0 && sensorArray[2] == 0)
  {
	  
    SerialMonitorInterface.print("left: 100");
  }
  if (sensorArray[0] == 0 && sensorArray[0] == 0 && sensorArray[2] == 1)
  {
	 
	  rotate(0.1,sRIGHT,1);
    SerialMonitorInterface.print("right: 001");
  }
  if (sensorArray[0] == 1 && sensorArray[0] == 0 && sensorArray[2] == 1)
  {
    int ra = random(0,1);
    if  (ra == 0);
    {
	  arc(0.4,sRIGHT,0.8);
    SerialMonitorInterface.print("turened right");
    }
    if (ra = 1);
    {
	  arc(0.8,sLEFT,0.4);
    SerialMonitorInterface.print("turened left");
    }
  }
  */
  
} 
/*******************************************************************/
void halt(float duration)
{
  drive(0, 0, duration);
}
/*******************************************************************/
void translate(float speed, String direction, float duration)
{
  SerialMonitorInterface.print("translate(");
  SerialMonitorInterface.print(speed);
  SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print(direction);
  SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print(duration);
  SerialMonitorInterface.println(")");

	if (direction == sREVERSE)
	{
		 speed *=-1;
	}
	drive(speed,speed,duration);
}
/*******************************************************************/
void arc(float speed, String direction, float duration)
{
	float speed_ratio = 0.075;
	float ls = speed;
	float rs = speed;

	if(direction == sLEFT)
	{
		ls *= speed_ratio;			
	}
	if (direction == sRIGHT)
	{
		rs *= speed_ratio;	
	}
	drive(ls,rs,duration);
}
/*******************************************************************/
void rotate(float speed, String direction, float duration)
{	
	float ls = speed;
	float rs = speed;


	if (direction == sLEFT)
	{
		ls *= -1;
		
	}
	if (direction == sRIGHT)
	{
		rs *= -1;
	}
	drive(ls, rs, duration);
}
/*******************************************************************/
void driveInit() {
  if (MOTOR_TYPE == DC_MOTOR)
  {
    dcMotorInit();
  }
  else
  {
    servoInit();
  }
}
/*******************************************************************/
void servoInit()
{
  Wire.begin();
  // while(!SerialMonitorInterface)//This will block until the Serial Monitor is opened on TinyScreen+/TinyZero platform!
  pinMode(9, OUTPUT);//Pin 9 is the reset pin for the servo controller TinyShield
  digitalWrite(9, LOW);
  delay(10);
  digitalWrite(9, HIGH);
  delay(100);
  
  
  if(servo.begin(20000))      //Set the period to 20000us or 20ms, correct for driving most servos
  {
    SerialMonitorInterface.println("Motor driver not detected!");
    while(1);
  }
  //The failsafe turns off the PWM output if a command is not sent in a certain amount of time.
  //Failsafe is set in milliseconds- comment or set to 0 to disable
  servo.setFailsafe(1000);
}
/*******************************************************************/
void dcMotorInit()
{
  //SerialMonitorInterface.begin(115200);
  Wire.begin();
  if (MOTOR_TYPE == DC_MOTOR) {
    DcMotorInit(maxPWM);
  } else {
    stepperInit();
  }
  delay(100);
  setMotorCurrent(100);
}
/*******************************************************************/
void drive(float left_proportion, float right_proportion, float delay_seconds)
{
  int left_speed;
  int right_speed;
  float duration = delay_seconds * 1000;

  if(MOTOR_TYPE == DC_MOTOR)
  {
    left_speed  = -left_proportion * maxPWM;
    right_speed = right_proportion * maxPWM;

    SerialMonitorInterface.print(", left_speed: ");
    SerialMonitorInterface.print(left_speed);
    SerialMonitorInterface.print(", right_speed: ");
    SerialMonitorInterface.println(right_speed);

    setDCMotor(LEFT_PORT, left_speed);
    setDCMotor(RIGHT_PORT, right_speed);
    delay(duration);
  }
  else
  {
    left_speed  = SERVO_HALT + left_proportion * MAX_SERVO_OFFSET;
    right_speed = SERVO_HALT - right_proportion * MAX_SERVO_OFFSET;
    
    SerialMonitorInterface.print(", left_speed: ");
    SerialMonitorInterface.print(left_speed);
    SerialMonitorInterface.print(", right_speed: ");
    SerialMonitorInterface.println(right_speed);

    servo.setServo(LEFT_PORT, left_speed);
    servo.setServo(RIGHT_PORT, right_speed);     
    delay(duration);
  }
  
  
}
float fmean(float arr[], int size) 
{
  if (size == 0) return 0.0;

  float sum = 0.0;
  for (int i = 0; i < size; i++) 
  {
    sum += arr[i];
    SerialMonitorInterface.print("sum: ");
    SerialMonitorInterface.println(sum);
  }
  SerialMonitorInterface.print("mean: ");
  SerialMonitorInterface.println(sum/size);
  return sum / size;
}

float fmode(float arr[], int size) 
{
  if (size == 0) return 0.0;

  float mode = arr[0];
  int maxCount = 1;

  for (int i = 0; i < size; i++) 
  {
    int count = 1;
    for (int j = i + 1; j < size; j++) 
    {
      if (arr[j] == arr[i]) 
      {
        count++;
      }
    }
    if (count > maxCount) 
    {
      maxCount = count;
      mode = arr[i];
    }
  }

  return mode;
}
/*******************************************************************/
