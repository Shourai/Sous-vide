//-------------------------------------------------------------------
// Sous-vide powered by Arduino
//------------------------------------------------------------------


// ************************************************
// Include relevant libraries
// ************************************************

// Libraries for the LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Libraries for the PID controller
#include <pid_controller.h>
#include <PID_v1.h>
#include <Timer.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
const int RelayPin = 9;

// One-Wire Temperature Sensor
const int ONE_WIRE_BUS = 7;

// ************************************************
// Rotary encoder variables
// ************************************************
const int PinCLK = 2;    // Used for generating interrupts using CLK signal
const int PinDT = 3;     // Used for reading DT signal
const int PinSW = 4;     // Used for the push button switch

// ************************************************
// Sensor Variables and constants
// ************************************************

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// PID variables and constants 
// ************************************************

double setpoint;
double input;
double output;

volatile long onTime = 0;

double Kp = 50;
double Ki = 0.2;
double Kd = 900;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

Timer t;

// ************************************************
// Setup and display initial screen
// ************************************************
void setup()
{
  Serial.begin(9600);

  // Turn on LCD backlight
  lcd.backlight();

  // Initialize LCD Display 
  lcd.begin(20, 4);
  lcd.setCursor(0, 0);
  lcd.print("   Sous Vide!");

  delay(1000);  // Splash screen
  lcd.clear();

  // Start up the DS18B20 One Wire Temperature Sensor
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0)) 
  {
    lcd.setCursor(0, 1);
    lcd.print("Sensor Error");
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);

  // Initialize the pins for the KY-040 rotary encoder
  pinMode(PinCLK, INPUT);
  pinMode(PinDT, INPUT);
  pinMode(PinSW, INPUT_PULLUP);
  attachInterrupt(0, Rotate, CHANGE); 

  // Initialize Relay Control:
  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, HIGH);  // make sure it is off to start

  // Initialize the PID and related variables
   myPID.SetTunings(Kp,Ki,Kd);
 
   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

    t.every(15, TempControl);
}

// ************************************************
// Main Control Loop
// ************************************************


void loop()
{ 
  LCD();
  TempSensor();
  t.update();
  Reset();
}

// ************************************************
// Initilialize the LCD
// ************************************************
void LCD()
{
  // Initialize LCD
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Current: ");
  lcd.print(input);
  lcd.write((char) 223);
  lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Settemp: ");
  lcd.print((int) setpoint);   //typecast to int to fit the display
  lcd.write((char) 223);
  lcd.print("C        ");

  delay(100); // 100 ms to update the LCD
}

// ************************************************
// Initialize the temperature sensor
// ************************************************
void TempSensor()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

myPID.SetMode(AUTOMATIC);
windowStartTime = millis();
myPID.SetTunings(Kp,Ki,Kd);

  myPID.Compute();
  onTime = output; 
}

// ************************************************
// Rotary encoder functions
// ************************************************

// Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
void Rotate()  {
  bool up;
  bool CLK = digitalRead(PinCLK);
  bool DT = digitalRead(PinDT);
  up = ((!CLK && DT) || (CLK && !DT));
  if (!up){
    setpoint += 1;
  }
  else {
    setpoint -= 1;
  }
}

// Reset rotary encoder when pushbutton is pressed
void Reset() 
{
  bool SW = digitalRead(PinSW);
  if ( SW == LOW ) {      // check if pushbutton is pressed
    setpoint = 0;              // if YES, then reset counter to ZERO
  }
}

// ************************************************
// PID functions
// ************************************************
void TempControl()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,LOW);
     Serial.println("Low");
  }
  else
  {
     digitalWrite(RelayPin,HIGH);
     Serial.println("High");
  }
  Serial.print(now);
  Serial.print(", ");
  Serial.print(onTime);
  Serial.print(", ");
  Serial.print(output);
  Serial.print(", ");
  Serial.print(windowStartTime);
  Serial.println("");
}
