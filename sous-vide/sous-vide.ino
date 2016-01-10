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

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 9

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS 7

// ************************************************
// Rotary encoder variables
// ************************************************
const int PinCLK = 2;    // Used for generating interrupts using CLK signal
const int PinDT = 3;     // Used for reading DT signal
const int PinSW = 4;     // Used for the push button switch


// ************************************************
// Display Variables and constants
// ************************************************

// define the degree symbol 
byte degree[8] = 
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

double Setpoint;
double Input;

// ************************************************
// Setup and display initial screen
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

   delay(1000);  // Splash screen
   lcd.clear();

    // Initialize the pins for the KY-040 rotary encoder
   pinMode(PinCLK, INPUT);
   pinMode(PinDT, INPUT);
   pinMode(PinSW, INPUT_PULLUP);
   attachInterrupt(0, Rotate, CHANGE); 
}

// ************************************************
// Main Control Loop
// ************************************************


void loop()
{ 
  LCD();
  TempSensor();
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
   lcd.print(Input);
   lcd.write(1);
   lcd.print("C");

   lcd.setCursor(0,1);
   lcd.print("Settemp: ");
   lcd.print((int) Setpoint);   //typecast to int to fit the display
   lcd.write(1);
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
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
}


// ************************************************
// Rotary encoder functions
// ************************************************

  // Interrupt service routine is executed when a HIGH to LOW transition is detected on CLK
  void Rotate()  {
  volatile boolean up;
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

  // Reset rotary encoder when pushbutton is pressed
  void Reset() {

  // define initital switch states
  int initSW = HIGH;
  int SW = HIGH;

	SW = digitalRead(PinSW);
	if ( SW == LOW && initSW == HIGH ) {      // check if pushbutton is pressed
		Setpoint = 0;              // if YES, then reset counter to ZERO
	}
	SW = initSW; // Reset the switch (SW) back to HIGH
}
