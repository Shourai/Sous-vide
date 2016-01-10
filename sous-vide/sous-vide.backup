//-------------------------------------------------------------------
//
// Customized sous vide for Arduino
// 
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 9

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS 7

// ************************************************
// PID Variables and constants
// ************************************************

// Rotary switch veriable
volatile int lastEncoded = 0;

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp = 250;
double Ki = 1;
double Kd = 20;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// rotary encoder
int SW;
int oldSW;

volatile boolean encChanged;
volatile boolean up;

// ************************************************
// DiSplay Variables and constants
// ************************************************
byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = RUN;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

const int PinCLK = 2;                   // Used for generating interrupts using CLK signal
const int PinDT = 3;                    // Used for reading DT signal
const int PinSW = 4;  



// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);
   
   // Turn on LCD backlight
   lcd.backlight();

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, HIGH);  // make sure it is off to start

   // Initialize LCD Display 

   lcd.begin(20, 4);
   lcd.createChar(1, degree); // create degree symbol from the binary
   
   lcd.setCursor(0, 0);
   lcd.print("   Sous Vide!");

   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print("Sensor Error");
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(3000);  // Splash screen
  lcd.clear();
   // Initialize the PID and related variables

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;



 	pinMode(PinCLK, INPUT);
	pinMode(PinDT, INPUT);
	pinMode(PinSW, INPUT_PULLUP);
	SW = HIGH;
	oldSW = HIGH;
	attachInterrupt(0, isr, FALLING); 

}



void isr()  {                    // Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
	volatile boolean CLK = digitalRead(PinCLK);
	volatile boolean DT = digitalRead(PinDT);
	up = ((!CLK && DT) || (CLK && !DT));
	if (!up){
		Setpoint += 1;
  }
	else {
		Setpoint -= 1;
  }
}


// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect){}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************


void loop()
{ 
  Run();
// Rotary encoder
	SW = digitalRead(PinSW);
	if ( SW == LOW && oldSW == HIGH) {      // check if pushbutton is pressed
		Setpoint = 0;              // if YES, then reset counter to ZERO
		encChanged = true;
	}
	oldSW = SW;

	if (encChanged)  {		    // do this only if rotation was detected
		encChanged = false;          // do NOT repeat IF loop until new rotation detected
	}
   
}
// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
   myPID.SetTunings(Kp,Ki,Kd);
      
   lcd.backlight();
     DoControl();
      
    lcd.setCursor(0,0);
    lcd.print("Current: ");
    lcd.print(Input);
    lcd.write(1);
    lcd.print("C");

   lcd.setCursor(0,1);
   lcd.print("Settemp: ");
   lcd.print((int) Setpoint);
   lcd.write(1);
   lcd.print("C        ");
      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
      }
 
      delay(100);
}
 


// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  
  // Execute control algorithm
  myPID.Compute();
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
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
  }
  else
  {
     digitalWrite(RelayPin,HIGH);
  }
}

