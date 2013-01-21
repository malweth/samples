/*             Test firmware for HexBright

Notes:
  Requires Arduino 1.0.1!

*/

#include <math.h>
#include <Wire.h>

// Settings
#define OVERTEMP                315

// Constants
#define ACC_ADDRESS             0x4C  // Accelerometer I2C Wire Address
#define ACC_REG_XOUT            0
#define ACC_REG_YOUT            1
#define ACC_REG_ZOUT            2
#define ACC_REG_TILT            3
#define ACC_REG_INTS            6
#define ACC_REG_MODE            7

// Pin assignments
#define DPIN_RLED               2    // RED LED & Switch (INPUT / OUTPUT)
#define DPIN_GLED               5    // GREEN LED (OUTPUT)
#define DPIN_PGOOD              7    // POWER GOOD (appears to be if switched on OR externally powered) (INPUT)
#define DPIN_PWR                8    // POWER REGULATOR (?) (OUTPUT)
#define DPIN_DRV_MODE           9    // MAIN LED DRIVER CURRENT LIMITER SWITCH (OUTPUT)
#define DPIN_DRV_EN             10   // Voltage Regulator Setting (? (OUTPUT)
#define DPIN_ACC_INT            3    // ACCELEROMETER INTERRUPT (INPUT)
#define APIN_TEMP               0    // TEMPERATURE (ANALOG INPUT)
#define APIN_CHARGE             3    // CHARGE (ANALOG INPUT)

// Interrupts
#define INT_SW                  0    // ??
#define INT_ACC                 1    // ??

// Modes
#define MODE_POWERUP            0
#define MODE_OFF                1
#define MODE_LOW                2
#define MODE_HIGH               3
#define MODE_KNOBBING           4
#define MODE_KNOBBED            5
#define MODE_BLINKING           6
#define MODE_BLINKING_PREVIEW   7
#define MODE_DAZZLING           8
#define MODE_DAZZLING_PREVIEW   9

// State
byte mode = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
boolean pluggedIn = false;
boolean serialDebug = false;

void setup()
{
  // We just powered on!  That means either we got plugged 
  // into USB, or (more likely) the user is pressing the 
  // power button.  We need to pull up the enable pin of 
  // the regulator very soon so we don't lose power.
  pinMode(DPIN_PWR,      INPUT);
  digitalWrite(DPIN_PWR, LOW);
//  pinMode(DPIN_PWR,      OUTPUT);
//  digitalWrite(DPIN_PWR, LOW);

  // Initialize GPIO
  pinMode(DPIN_RLED,     INPUT);
  pinMode(DPIN_GLED,     OUTPUT);
  pinMode(DPIN_DRV_MODE, OUTPUT);
  pinMode(DPIN_DRV_EN,   OUTPUT);
  pinMode(DPIN_ACC_INT,  INPUT);
  pinMode(DPIN_PGOOD,    INPUT);
  digitalWrite(DPIN_DRV_MODE, LOW);
  digitalWrite(DPIN_DRV_EN,   LOW);
  digitalWrite(DPIN_ACC_INT,  HIGH);
  
  // Initialize serial busses
  Serial.begin(9600);
  Wire.begin();

  // Configure accelerometer
  byte config[] = {
    ACC_REG_INTS,  // First register (see next line)
    0xE4,  // Interrupts: shakes, taps
    0x00,  // Mode: not enabled yet
    0x00,  // Sample rate: 120 Hz
    0x0F,  // Tap threshold
    0x10   // Tap debounce samples
  };
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(config, sizeof(config));
  Wire.endTransmission();

  // Enable accelerometer
  byte enable[] = {ACC_REG_MODE, 0x01};  // Mode: active!
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(enable, sizeof(enable));
  Wire.endTransmission();
  
  btnTime = millis();
  btnDown = digitalRead(DPIN_RLED);
  mode = MODE_OFF;

  Serial.println("Powered up!");
}

void loop()
{
  static unsigned long lastTime, lastTempTime, lastAccTime;
  static float lastKnobAngle, knob;
  static byte blink;
  unsigned long time = millis();
  int charge = analogRead(APIN_CHARGE);
  
  if (!serialDebug)
  {
    if (charge < 128) // Charging
    {
       digitalWrite(DPIN_GLED, LOW);
       pinMode(DPIN_RLED, OUTPUT);
       digitalWrite(DPIN_RLED, HIGH);
       pluggedIn = true;
    }
    else if (charge > 768) // Charged
    {
       digitalWrite(DPIN_GLED, HIGH);
       pinMode(DPIN_RLED, OUTPUT);
       digitalWrite(DPIN_RLED, LOW);
       pluggedIn = true;
    }
    else if (pluggedIn) // Battery, state change only
    {
      digitalWrite(DPIN_GLED, LOW);
      digitalWrite(DPIN_RLED, LOW);
      pinMode(DPIN_RLED, INPUT);
      pluggedIn = false;
    }
  }
  else if (pluggedIn)  // Debugs on, emulate non-plugged in state
  {
    digitalWrite(DPIN_GLED, LOW);
    digitalWrite(DPIN_RLED, LOW);
    pinMode(DPIN_RLED, INPUT);
    pluggedIn = false;
  }

  // Only print debugs if requested (press 'd' at Serial Input)
  if (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
      case 'd':
      {
        serialDebug = true;
      }
      break;

      case 'x':
      {
        serialDebug = false;
      }
      break;
    }
  }

  // Check the temperature sensor
  if (time-lastTempTime > 1000)
  {
    lastTempTime = time;
    int temperature = analogRead(APIN_TEMP);
    int charge = analogRead(APIN_CHARGE);
    if (temperature > OVERTEMP)
    {
      Serial.println("Overheat shutdown!");
      mode = MODE_OFF;
      digitalWrite(DPIN_DRV_MODE, LOW);
      digitalWrite(DPIN_DRV_EN, LOW);
      digitalWrite(DPIN_PWR, LOW);
    }
    else if (serialDebug)  // If debug printouts are requested, print them.
    {
      Serial.print("Temperature = ");
      Serial.print(temperature);
      Serial.print("; Charge = ");
      Serial.print(charge);
      
     char accel[3];
     readAccel(accel);
     Serial.print("; Acceleration = ");
     Serial.print(accel[0], DEC);
     Serial.print(", ");
     Serial.print(accel[1], DEC);
     Serial.print(", ");
     Serial.print(accel[2], DEC);
     
     byte pgood = digitalRead(DPIN_PGOOD);
     Serial.print("; LED driver power good = ");
     Serial.print(pgood?"Yes":"No");
     
     Serial.println();
    }
  }

  // Check for Taps & Shakes for blink / strobe mode
  // Check if the accelerometer wants to interrupt
  byte tapped = 0, shaked = 0;
  if (!digitalRead(DPIN_ACC_INT))
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_TILT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 1);  // This one stops.
    byte tilt = Wire.read();
    
    if (time-lastAccTime > 500)
    {
      lastAccTime = time;
  
      tapped = !!(tilt & 0x20);
      shaked = !!(tilt & 0x80);
  
      if (serialDebug)
      {
        if (tapped) Serial.println("Tap!");
        if (shaked) Serial.println("Shake!");
      }
    }
  }

  // Perform actions based on mode (state)
  switch (mode)
  {
    case MODE_KNOBBING:
    {
      if (time-lastTime < 100) break;
      lastTime = time;
      
      char accel[3];
      readAccel(accel);
      float knobVal = accel[1];
      
      knobVal = knobVal + 25;    // Varies approx from [-22, 22] - read on a 0-50 scale.

      knob = knobVal * 5 + 5;
      if (knob < 0)
        knob = 0;
      if (knob > 255)
        knob = 255;

      // Make apparent brightness changes linear by squaring the
      // value and dividing back down into range.  This gives us
      // a gamma correction of 2.0, which is close enough.
      byte bright = (long)(knob * knob) >> 8;
      // Avoid ever appearing off in this mode!
      if (bright < 8) bright = 8;
      analogWrite(DPIN_DRV_EN, bright);
  
      if (serialDebug)
      {
        Serial.print("Knob = ");
        Serial.print(knob);
        Serial.print("\tBright = ");
        Serial.println(bright);
      }
    }
    break;
    
    case MODE_BLINKING:
    case MODE_BLINKING_PREVIEW:
    {
      if (time-lastTime < 250) break;
      lastTime = time;

      blink = !blink;
      digitalWrite(DPIN_DRV_EN, blink);
    }
    break;
    
    case MODE_DAZZLING:
    case MODE_DAZZLING_PREVIEW:
    {
      if (time-lastTime < 10) break;
      lastTime = time;
    
      digitalWrite(DPIN_DRV_EN, random(4)<1);
    }
    break;
  }
  
  // Check for mode changes
  byte newMode = mode;
  byte newBtnDown;
  if (pluggedIn)
    newBtnDown = LOW;
  else
    newBtnDown = digitalRead(DPIN_RLED);
  switch (mode)
  {
    case MODE_OFF:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_LOW;
      if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
        newMode = MODE_KNOBBING;
    }
    break;

    case MODE_LOW:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_HIGH;
      if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
        newMode = MODE_KNOBBING;
    }
    break;

    case MODE_HIGH:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_OFF;
      if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
        newMode = MODE_KNOBBING;
    }
    break;
 
    case MODE_KNOBBING:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_KNOBBED;
      if (btnDown && newBtnDown && tapped)
        newMode = MODE_BLINKING_PREVIEW;
    }
    break;

    case MODE_KNOBBED:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_OFF;
      if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
        newMode = MODE_KNOBBING;
    }
    break;

    case MODE_BLINKING:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_OFF;
      if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
        newMode = MODE_BLINKING_PREVIEW;
    }
    break;
    
    case MODE_BLINKING_PREVIEW:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_BLINKING;
      if (btnDown && newBtnDown && tapped)
        newMode = MODE_DAZZLING_PREVIEW;
    }
    break;
    
    case MODE_DAZZLING:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_OFF;
      if (btnDown && newBtnDown && (time-btnTime)>500)  // Held
        newMode = MODE_DAZZLING_PREVIEW;
    }
    break;

    case MODE_DAZZLING_PREVIEW:
    {
      if (btnDown && !newBtnDown)  // Button released
        newMode = MODE_DAZZLING;
      if (btnDown && newBtnDown && tapped)
        newMode = MODE_BLINKING_PREVIEW;
    }
    break;
  }

  // Do the mode transitions
  if (newMode != mode)
  {
    switch (newMode)
    {
      case MODE_OFF:
      {
        Serial.println("Mode = off");
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, LOW);
        digitalWrite(DPIN_DRV_MODE, LOW);
        digitalWrite(DPIN_DRV_EN, LOW);
      }
      break;

      case MODE_LOW:
      {
        Serial.println("Mode = low");
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, HIGH);
        digitalWrite(DPIN_DRV_MODE, LOW);
        analogWrite(DPIN_DRV_EN, 255);
      }
      break;

      case MODE_HIGH:
      {
        Serial.println("Mode = high");
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, HIGH);
        digitalWrite(DPIN_DRV_MODE, HIGH);
        analogWrite(DPIN_DRV_EN, 255);
      }
      break;
      
      case MODE_KNOBBING:
      {
        Serial.println("Mode = knobbing");
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, HIGH);
        knob = (mode==MODE_OFF) ? 0 : 255;
      }
      break;

      case MODE_KNOBBED:
      {
        Serial.println("Mode = knobbed");
      }
      break;
      
      case MODE_BLINKING:
      case MODE_BLINKING_PREVIEW:
      {
        Serial.println("Mode = blinking");
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, HIGH);
        digitalWrite(DPIN_DRV_MODE, LOW);
      }
      break;
      
      case MODE_DAZZLING:
      case MODE_DAZZLING_PREVIEW:
      {
        Serial.println("Mode = dazzling");
        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, HIGH);
        digitalWrite(DPIN_DRV_MODE, HIGH);
      }
      break;
    }

    mode = newMode;
  }

  // Remember button state so we can detect transitions
  if (newBtnDown != btnDown)
  {
    btnTime = time;
    btnDown = newBtnDown;
    delay(50);
  }
}

void readAccel(char *acc)
{
  while (1)
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_XOUT);
    Wire.endTransmission(false);       // End transmission, (but seize the wire to prevent simultaneous accesses)
    Wire.requestFrom(ACC_ADDRESS, 3);  // Send Request for 3 bytes to I2C, (stop flag defaults to true).

    for (int i = 0; i < 3; i++)
    {
      if (!Wire.available())
        continue;
      acc[i] = Wire.read();
      if (acc[i] & 0x40)  // Indicates failed read; redo!
        continue;
      if (acc[i] & 0x20)  // Sign-extend
        acc[i] |= 0xC0;
    }
    break;
  }
}

