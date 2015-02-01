// Time Lapse Rail Controller
// by Mark Needham

#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <EEPROM.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define BUTTON_SHIFT BUTTON_SELECT

const int pDelayAddress = 0;
const int pWidthAddress = 8;

int aDirPin = 12;
int aPWMPin = 3;
int aBrakePin = 9;
int aCurrentPin = 0;
int bDirPin = 13;
int bPWMPin = 11;
int bBrakePin = 8;
int bCurrentPin = 1;

long pulseT = 25;
long pulseInt = 1500;
long brakeT = 0; 


unsigned long now = 0;
unsigned long lastInput = 0;

enum operatingState { 
  OFF = 0, SET_PW, SET_PINT, SET_TIME, MAN, RUN };
operatingState opState = OFF;

enum motorDirection { 
  LEFT = 0, RIGHT };
motorDirection mDir = LEFT;

class Motor
{
  int motorPin;
  int dirPin;
  int brakePin;
  long pulseTime;
  long pulseDelay;
  long brakeTime;
  int AutoRun;
  long currentStep;

  int motorState;
  int motorDir;
  int brakeState;
  unsigned long previousMillis;

public:
  Motor(int mPin, int dPin, int bPin, long on, long pDel, long bTime)
  {
    motorPin = mPin;
    pinMode(motorPin, OUTPUT);

    dirPin = dPin;
    pinMode(dirPin, OUTPUT);

    brakePin = bPin;
    pinMode(brakePin, OUTPUT);

    pulseTime = on;
    pulseDelay = pDel;
    brakeTime = bTime;

    AutoRun = LOW;
    motorState = LOW;
    motorDir = LEFT;
    previousMillis = 0;
    currentStep = 0;

  }

  void Update()
  {
    unsigned long currentMillis = millis();

    if ((brakeState == HIGH) && (currentMillis - previousMillis >= pulseTime + brakeTime))
    {
      brakeState = LOW;
      digitalWrite( brakePin, brakeState);
    }
    if ((motorState == HIGH) && (currentMillis - previousMillis >= pulseTime))
    {
      motorState = LOW;
      //      previousMillis = currentMillis;  // may not need this
      digitalWrite( motorPin, motorState);
      if (brakeTime >= 10) {
        brakeState = HIGH;
        digitalWrite( brakePin, brakeState);
      }
    }
    else if ((AutoRun == HIGH) && (motorState == LOW) && (currentMillis - previousMillis >= pulseDelay))
    {
      motorState = HIGH;
      previousMillis = currentMillis;
      digitalWrite( motorPin, motorState);
      currentStep++;
    }
  }

  void direction(int d)
  {
    digitalWrite(dirPin, d);
  }

  void Auto(boolean r)
  {
    AutoRun = r;
  }

  uint16_t stepCount()
  {
    return currentStep;
  }

};

Motor motor1(aPWMPin, aDirPin, aBrakePin, pulseT, pulseInt, brakeT);

void setup() {
  // Debugging output
  Serial.begin(9600);
  // setup the LCD's number of columns and rows:
  lcd.begin(16,2);

  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  int time = millis();
  lcd.print(F("Time Lapse Rail"));
  lcd.setCursor(0,1);
  lcd.print(F("   Controller   "));
  time = millis() - time;
  Serial.print("Took "); 
  Serial.print(time); 
  Serial.println(" ms");
  lcd.setBacklight(WHITE);
}

void loop() {
  {
    uint8_t buttons = 0;
    buttons = ReadButtons();
    
    if (buttons & BUTTON_LEFT) {
      motor1.direction(LEFT);
    }
    if (buttons & BUTTON_RIGHT) {
      motor1.direction(RIGHT);
    }
    if (buttons & BUTTON_UP) {
      opState = RUN;
      motor1.Auto(HIGH);

      lcd.clear();
      lcd.print(pulseT);
      lcd.print("  ");
      lcd.print(pulseInt);
      lcd.print("  ");
      lcd.print(brakeT);

      lcd.setCursor(0,1);
      lcd.print(motor1.stepCount());
      lcd.setCursor(15,1);
      lcd.print("R");
    }
    if (buttons & BUTTON_DOWN) {
      opState = OFF;
      motor1.Auto(LOW);
      lcd.setCursor(0,0);
      lcd.print("    Stopped    ");

      lcd.setCursor(15,1);
      lcd.print(" ");
    }
    if (opState == RUN) {
      lcd.setCursor(0,1);
      lcd.print(motor1.stepCount());
    }
    motor1.Update();
  }
}


uint8_t ReadButtons()
{
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
}

/*void setMotor (int speed, boolean reverse)
{
  digitalWrite(aDirPin, reverse);
  analogWrite(aPWMPin, speed);
}
*/

// Off mode
void Off()
{
  lcd.setBacklight(0);
  uint8_t buttons = 0;
  while(!(buttons & (BUTTON_RIGHT)))
  {
    buttons = ReadButtons();
  }
  opState = RUN; // start control
}

// ***************
// Set Pulse width
// ***************
void Set_PulseWidth()
{
  lcd.setBacklight(WHITE);
  lcd.print(F("Set Pulse Width:"));
  uint8_t buttons = 0;
  while(true)
  {
    buttons = ReadButtons();

    int increment = 1;
    if (buttons & BUTTON_SHIFT)
    {
      increment *= 10;
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = RUN;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = SET_PINT;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      pulseT += increment;
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      pulseT -= increment;
      delay(200);
    }
    if ((millis() - lastInput) > 3000) // return to RUN after 3 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0,1);
    lcd.print(pulseT);
    lcd.print(" ");
    //DoControl();
  }
}

// ******************
// Set pulse interval
// ******************
void Set_PulseInterval()
{
  lcd.setBacklight(WHITE);
  lcd.print(F("  Set Interval:"));

  uint8_t buttons = 0;
  while(true)
  {
    buttons = ReadButtons();

    int increment = 1;
    if (buttons & BUTTON_SHIFT)
    {
      increment *= 10;
    }
    if (buttons & BUTTON_LEFT)
    {
      opState = SET_PW;
      return;
    }
    if (buttons & BUTTON_RIGHT)
    {
      opState = SET_TIME;
      return;
    }
    if (buttons & BUTTON_UP)
    {
      pulseInt += increment;
      delay(200);
    }
    if (buttons & BUTTON_DOWN)
    {
      pulseInt -= increment;
      delay(200);
    }
    if ((millis() - lastInput) > 3000) // return to RUN after 3 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0,1);
    lcd.print(pulseInt);
    lcd.print(" ");
    //DoControl();
  }
}


// defaults
// pulseT = 25
// pulseInt = 1500
// shotCountsOrTimedMode = whatever

/*
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 500;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
*/




