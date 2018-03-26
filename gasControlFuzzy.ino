#include <EEPROMex.h>						// https://github.com/thijse/Arduino-EEPROMEx

#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>				// https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library
#include <utility/Adafruit_MCP23017.h>

#include <OneWire.h>						// https://github.com/bigjosh/OneWireNoResistor
#include <DallasTemperature.h>					// https://github.com/milesburton/Arduino-Temperature-Control-Library


#include <AccelStepper.h>					// https://github.com/adafruit/AccelStepper
#include <Adafruit_MotorShield.h>				// https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <FuzzyRule.h>						// https://github.com/zerokol/eFLL
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

/*-----( Definitions )-----*/

#define ONE_WIRE_BUS 2						// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_PWR 3						// Use GPIO pins for power/ground to simplify the wiring
#define ONE_WIRE_GND 4						// Use GPIO pins for power/ground to simplify the wiring

#define PUMP_WIRE_PWR 5						// Use GPIO pins for power/ground to simplify the wiring
#define PUMP_WIRE_GND 6						// Use GPIO pins for power/ground to simplify the wiring

#define BUZZER_WIRE_PWR 9

#define LCD_BUFFER_SIZE 4
#define DISPLAY_TARGET_DECIMALS 0
#define DISPLAY_ACTUAL_DECIMALS 1

//#define OFF 0x0						// These #defines make it easy to set the backlight color
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

#define SENSOR_PRECISION 12

#define STEPPER_SPEED 10

#define BUZZER_FREQUENCY 750

#define READ_TEMP_SENSORS_EVERY 1000
#define READ_USER_INPUT_EVERY 20
#define WRITE_DISPLAY_EVERY 50
#define COMPUTE_FUZZY_EVERY 5000

/*-----( Constants )-----*/

const int CONFIG_VERSION = 0.7;
const int memoryBase = 32;

/*----( Objects )----*/

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();		// Create a RGB LCD instance

OneWire oneWire(ONE_WIRE_BUS);					// Create a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);				// Pass our oneWire reference to Dallas Temperature.
DeviceAddress tempSensor;					// Arrays to hold device address

Adafruit_MotorShield AFMS = Adafruit_MotorShield();		// Create a motor shield object with the default I2C address
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 2);	// Connect a stepper motor with 200 steps per revolution (1.8 degree) to motor port #2 (M3 and M4)

void forwardstep() {
  myStepper->onestep(FORWARD, MICROSTEP);			// Anti-clockwise
}

void backwardstep() {
  myStepper->onestep(BACKWARD, MICROSTEP);			// Clockwise
}

AccelStepper stepper(forwardstep, backwardstep);		// Wrap the stepper in an AccelStepper object

enum operatingState { OFF = 0, SETP, MAN, AUTO };
enum pumpState { NORECIRC = 0, RECIRC };

struct manualSettings {
  int setPoint;
  pumpState pState;
};

struct gasDialSettings {
  int currentPosition;
  int minPosition;
  int maxPosition;
};

//typedef struct manualSettings manSettings;

struct allSettings {
  int version;
  manualSettings manSettings;
  gasDialSettings gasSettings;
  double setPoint;
  double fuzzyScale;
};

// Place default values for settings here
allSettings settings = { CONFIG_VERSION, { 65, RECIRC }, {0, 34, 70 }, 55, -0.25 };

/*----( Global Variables )----*/

// EEPROM
int configAddress;

//
operatingState opState = OFF;
boolean opStateChanged = true;
char *StrOpState[] = { "     OFF      ", "    SETUP     ", "    MANUAL    ", "     AUTO     " };

operatingState selectedState = OFF;
boolean selectedStateChanged = true;

// User Input
uint8_t lastButtonPressed = 0;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];// temporary array for use when parsing
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
double doubleFromPC = 0.0;
boolean newData = false;

boolean positionsSet = false;

// Logic
double actual = 0;
double error = -100;

// LCD
char lcd_buffer[LCD_BUFFER_SIZE];             			// LCD buffer used for the better string format to LCD
bool targetChanged = true;
bool actualChanged = true;
uint8_t lastBacklight = OFF;

byte degree[8] = 						// define the degree symbol
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

// Stepper motor
int jogSize = 16;

// Processing
Fuzzy* fuzzy = new Fuzzy();

// Loop Timing
unsigned long lastTemperatureRead = 0;
unsigned long lastFuzzyCompute = 0;
unsigned long lastDisplayWrite = 0;
unsigned long lastUserInput = 0;
unsigned long lastBlinkLCD = 0;

void setup() {

  Serial.begin(9600);
  Serial.println("Stubby's Gas Control");

  initBuzzer();
  initDisplay();
  initTempSensor();
  initFuzzyLogic();
  initStepper();

}

void doFunctionAtInterval(void (*callBackFunction)(), unsigned long *lastEvent, unsigned long Interval) {

  unsigned long now = millis();

  if ((now - *lastEvent) >= Interval)
  {

    callBackFunction();
    *lastEvent = now;

  }

}

void loop() {

  // read user input
  doFunctionAtInterval(readUserInput, &lastUserInput, READ_USER_INPUT_EVERY);

  // read temperature sensors
  doFunctionAtInterval(readTempSensor, &lastTemperatureRead, READ_TEMP_SENSORS_EVERY);

  // write display values
  doFunctionAtInterval(updateDisplay, &lastDisplayWrite, WRITE_DISPLAY_EVERY);

  // process fuzzy logic
  doFunctionAtInterval(processFuzzyLogic, &lastFuzzyCompute, COMPUTE_FUZZY_EVERY);
  
  turnDial();

}

void initBuzzer() {

  pinMode(BUZZER_WIRE_PWR, OUTPUT);

  tone(BUZZER_WIRE_PWR, BUZZER_FREQUENCY);
  delay(20);
  noTone(BUZZER_WIRE_PWR);

}

void initDisplay() {

  lcd.begin(16, 2);
  lcd.clear();

  lcd.createChar(1, degree); 					// create degree symbol from the binary

  lcd.setBacklight(WHITE);

  lcd.print(F(" Stubbydrainer's"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Brewduino!"));

  delay(3000);							// Splash screen delay

}

void initTempSensor() {

  sensors.begin();

  Serial.print("Locating temperature sensor...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);			// locate devices on the bus
  Serial.println(" devices.");

  if (!sensors.getAddress(tempSensor, 0)) {

    Serial.println("Unable to find address for Device 0");
    displayError("   No sensor!    ");

  } else {

    sensors.setResolution(tempSensor, SENSOR_PRECISION);
    sensors.requestTemperatures();				// prime the pump for the next one - but don't wait

    Serial.println("Success!");

  }

}

void initStepper() {

  AFMS.begin(); // Start the bottom shield

  stepper.setMaxSpeed(200.0);
  stepper.setAcceleration(100.0);
  stepper.setCurrentPosition(0);

  settings.gasSettings.currentPosition = 0;

}

void initSettings() {

  allSettings tempSettings;
  int timeItTook = 0;

  EEPROM.setMemPool(memoryBase, EEPROMSizeMega);
  configAddress = EEPROM.getAddress(sizeof(allSettings));
  timeItTook = EEPROM.readBlock(configAddress, tempSettings);	// Read EEPROM settings to temporary location to compare CONFIG_VERSION

  // Update EEPROM from new settings configuration if necessary
  if (tempSettings.version != CONFIG_VERSION)
  {
    timeItTook = EEPROM.writeBlock(configAddress, settings);	// Settings have not been saved before or settings configuration has changed
  }

  timeItTook = EEPROM.readBlock(configAddress, settings);	// Read settings from EEPROM

}

void initFuzzyLogic() {

  // FuzzyInput error equals distance from current temp to target temp
  FuzzyInput* error = new FuzzyInput(1);

  FuzzySet* ErrorNeg = new FuzzySet(-100, -100, -10, 0);
  FuzzySet* ErrorZero = new FuzzySet(-10, 0, 0, 10);
  FuzzySet* ErrorPos = new FuzzySet(0, 10, 100, 100);

  error->addFuzzySet(ErrorNeg);
  error->addFuzzySet(ErrorZero);
  error->addFuzzySet(ErrorPos);

  fuzzy->addFuzzyInput(error);

  // FuzzyInput errorChange = rate of change of temp in degrees per second
  FuzzyInput* errorChange = new FuzzyInput(2);

  FuzzySet* ChangeNeg = new FuzzySet(-5, -5, -1, 0);
  FuzzySet* ChangeZero = new FuzzySet(-2, -1, 1, 2);
  FuzzySet* ChangePos = new FuzzySet(0, 1, 5, 5);

  errorChange->addFuzzySet(ChangeNeg);
  errorChange->addFuzzySet(ChangeZero);
  errorChange->addFuzzySet(ChangePos);

  fuzzy->addFuzzyInput(errorChange);

  // FuzzyOutput
  FuzzyOutput* flowChange = new FuzzyOutput(1);

  FuzzySet* LargeDecrease = (-10, -10, -4, -3);
  FuzzySet* SmallDecrease = (-5, -4, -2, -1);
  FuzzySet* ZeroChange = (-2, -1, 1, 2);
  FuzzySet* SmallIncrease = (1, 2, 4, 5);
  FuzzySet* LargeIncrease = (3, 4, 10, 10);

  flowChange->addFuzzySet(LargeDecrease);
  flowChange->addFuzzySet(SmallDecrease);
  flowChange->addFuzzySet(ZeroChange);
  flowChange->addFuzzySet(SmallIncrease);
  flowChange->addFuzzySet(LargeIncrease);

  fuzzy->addFuzzyOutput(flowChange);


  FuzzyRuleConsequent* thenLargeDecrease = new FuzzyRuleConsequent();
  thenLargeDecrease->addOutput(LargeDecrease);
  FuzzyRuleConsequent* thenSmallDecrease = new FuzzyRuleConsequent();
  thenSmallDecrease->addOutput(SmallDecrease);
  FuzzyRuleConsequent* thenZeroChange = new FuzzyRuleConsequent();
  thenZeroChange->addOutput(ZeroChange);
  FuzzyRuleConsequent* thenSmallIncrease = new FuzzyRuleConsequent();
  thenSmallIncrease->addOutput(SmallIncrease);
  FuzzyRuleConsequent* thenLargeIncrease = new FuzzyRuleConsequent();
  thenLargeIncrease->addOutput(LargeIncrease);

  //Rules

  // If error is zero and change is zero, then flowchange is zero
  FuzzyRuleAntecedent* ifErrorZeroANDChangeZero = new FuzzyRuleAntecedent();
  ifErrorZeroANDChangeZero->joinWithAND(ErrorZero, ChangeZero);
  FuzzyRule* fuzzyRule1 = new FuzzyRule(1, ifErrorZeroANDChangeZero, thenZeroChange);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // If error is zero and change is neg, then flowchange is small increase
  FuzzyRuleAntecedent* ifErrorZeroANDChangeNeg = new FuzzyRuleAntecedent();
  ifErrorZeroANDChangeNeg->joinWithAND(ErrorZero, ChangeNeg);
  FuzzyRule* fuzzyRule2 = new FuzzyRule(2, ifErrorZeroANDChangeNeg, thenSmallIncrease);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // If error is zero and change is pos, then flowchange is small decrease
  FuzzyRuleAntecedent* ifErrorZeroANDChangePos = new FuzzyRuleAntecedent();
  ifErrorZeroANDChangePos->joinWithAND(ErrorZero, ChangePos);
  FuzzyRule* fuzzyRule3 = new FuzzyRule(3, ifErrorZeroANDChangePos, thenSmallDecrease);
  fuzzy->addFuzzyRule(fuzzyRule3);

  // IF error is negative and change is zero, then flowchange is small increase
  FuzzyRuleAntecedent* ifErrorNegANDChangeZero = new FuzzyRuleAntecedent();
  ifErrorNegANDChangeZero->joinWithAND(ErrorNeg, ChangeZero);
  FuzzyRule* fuzzyRule4 = new FuzzyRule(4, ifErrorNegANDChangeZero, thenSmallIncrease);
  fuzzy->addFuzzyRule(fuzzyRule4);

  // If error is negative and change is neg, then flowchange is large increase
  FuzzyRuleAntecedent* ifErrorNegANDChangeNeg = new FuzzyRuleAntecedent();
  ifErrorNegANDChangeNeg->joinWithAND(ErrorNeg, ChangeNeg);
  FuzzyRule* fuzzyRule5 = new FuzzyRule(5, ifErrorNegANDChangeNeg, thenLargeIncrease);
  fuzzy->addFuzzyRule(fuzzyRule5);

  // if error is negative and change is positive, then flowchange is zero
  FuzzyRuleAntecedent* ifErrorNegANDChangePos = new FuzzyRuleAntecedent();
  ifErrorNegANDChangePos->joinWithAND(ErrorNeg, ChangePos);
  FuzzyRule* fuzzyRule6 = new FuzzyRule(6, ifErrorNegANDChangePos, thenZeroChange);
  fuzzy->addFuzzyRule(fuzzyRule6);

  // if error is positive and change is zero, then flowchange is small decrease
  FuzzyRuleAntecedent* ifErrorPosANDChangeZero = new FuzzyRuleAntecedent();
  ifErrorPosANDChangeZero->joinWithAND(ErrorPos, ChangeZero);
  FuzzyRule* fuzzyRule7 = new FuzzyRule(7, ifErrorPosANDChangeZero, thenSmallDecrease);
  fuzzy->addFuzzyRule(fuzzyRule7);

  // if erro is positive and change is positive, then flow is large decrease
  FuzzyRuleAntecedent* ifErrorPosANDChangePos = new FuzzyRuleAntecedent();
  ifErrorPosANDChangePos->joinWithAND(ErrorPos, ChangePos);
  FuzzyRule* fuzzyRule8 = new FuzzyRule(8, ifErrorPosANDChangePos, thenLargeDecrease);
  fuzzy->addFuzzyRule(fuzzyRule8);

  // if error is positive and change is negative, then flowchange is zero
  FuzzyRuleAntecedent* ifErrorPosANDChangeNeg = new FuzzyRuleAntecedent();
  ifErrorPosANDChangeNeg->joinWithAND(ErrorPos, ChangeNeg);
  FuzzyRule* fuzzyRule9 = new FuzzyRule(9, ifErrorPosANDChangeNeg, thenZeroChange);
  fuzzy->addFuzzyRule(fuzzyRule9);

}

void readUserInput() {

  uint8_t buttons = readButtons();

  if (buttons) {

    switch (opState) {
      case OFF:

      case SETP:

      case MAN:
      /*if (buttons & BUTTON_UP)
        {
        settings.manSettings.setPoint = min(102,settings.manSettings.setPoint + 1);
        targetChanged = true;
        }
        if (buttons & BUTTON_DOWN)
        {
        settings.manSettings.setPoint = max(0,settings.manSettings.setPoint - 1);
        targetChanged = true;
        }
        if (buttons & BUTTON_SELECT)
        {
        updateSettings();
        }*/
      case AUTO:

      default:
        //do nothing
        break;
    }

  }

  if (buttons) {

    if (buttons & BUTTON_LEFT) {

      selectedState = max(0, int(selectedState) - 1);
      selectedStateChanged = true;

    }

    if (buttons & BUTTON_RIGHT) {

      selectedState = min(3, int(selectedState) + 1);
      selectedStateChanged = true;

    }

    if (buttons & BUTTON_SELECT) {

      opState = selectedState;
      opStateChanged = true;
      actualChanged = true;

    }

  }

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';// terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseUserInput();
    newData = false;
  }

}

void parseUserInput() {

  // split the data into its parts
  char * strtokIndx;	// this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");	// get the first part - the string
  strcpy(messageFromPC, strtokIndx);	// copy it to messageFromPC

  if (String(messageFromPC) == "L") {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LEFT ");
    stepper.moveTo(stepper.currentPosition() - jogSize);
    return;
  }

  if (String(messageFromPC) == "R") {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RIGHT");
    stepper.moveTo(stepper.currentPosition() + jogSize);
    return;
  }

  if (String(messageFromPC) == "MAX") {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MAX SET");
    settings.gasSettings.maxPosition = stepper.currentPosition();
    return;
  }

  if (String(messageFromPC) == "MIN") {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MIN SET");
    settings.gasSettings.minPosition = stepper.currentPosition();
    return;
  }

  if (String(messageFromPC) == "UPDATE") {
    lcd.clear();
    lcd.setCursor(0, 0);
    updateSettings();
    lcd.print("SETTINGS UPDATED");
    positionsSet = true;
    return;
  }

  if (String(messageFromPC) == "DIAL") {

    double difference;
    strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
    doubleFromPC = atof(strtokIndx);
    stepper.moveTo(doubleFromPC * jogSize);
    return;
  }

  else Serial.println ("Command not recognised.");

}

void turnDial () {

  stepper.run();
  settings.gasSettings.currentPosition = stepper.currentPosition();

}

uint8_t readButtons() {

  uint8_t buttons = lcd.readButtons();

  if (buttons) {

    tone(BUZZER_WIRE_PWR, BUZZER_FREQUENCY);
    delay(20);
    noTone(BUZZER_WIRE_PWR);

    lastButtonPressed = buttons;
    return 0;							//Wait until button is release before sending

  }
  else {

    buttons = lastButtonPressed;
    lastButtonPressed = 0;
    return buttons;						//Button is released - send!

  }

}

void readTempSensor() {

  double previousActual = actual;
  actual = double(sensors.getTempC(tempSensor));

  if (previousActual != actual) {
    actualChanged = true;
  }
  sensors.requestTemperatures(); 				// prime the pump for the next one - but don't wait

}

void processFuzzyLogic() {

  double previousError = error;
  Serial.print("previousError: "); Serial.println(previousError, 2);

  error = actual - settings.setPoint;
  Serial.print("Error: "); Serial.println(error, 2);
  Serial.print("Difference: "); Serial.println(error - previousError, 2);
  fuzzy->setInput(1, error);
  fuzzy->setInput(2, (error - previousError) / (COMPUTE_FUZZY_EVERY / 1000.0));

  fuzzy->fuzzify();

  //double output = double(fuzzy->defuzzify(1));
  double output = fuzzy->defuzzify(1);
  Serial.print("Output: "); Serial.println(output, 2);

  double tPosition = settings.gasSettings.currentPosition + (output * 100 / (settings.gasSettings.maxPosition - settings.gasSettings.minPosition));
  stepper.moveTo(tPosition);
  //ValveSetPoint = ValveSetPoint + (output * 100 / valve.getCycleTime());

}

void updateDisplay() {

  if (selectedStateChanged) {

    lcd.setCursor(0, 0);
    lcd.print((selectedState == OFF ? " " : "<"));
    lcd.print(StrOpState[int(selectedState)]);
    lcd.print((selectedState == AUTO ? " " : ">"));

    //lcd.setCursor(0, 1);
    //lcd.print("                ");

  }

  switch (opState) {

    case OFF:

      if (actualChanged) {

        lcd.setCursor(0, 1);
        lcd.print("   A=");
        dtostrf(actual, 5, DISPLAY_ACTUAL_DECIMALS, lcd_buffer);
        lcd.print(lcd_buffer);
        lcd.write(1);
        lcd.print("c    ");

      }

      break;

    case SETP:

      lcd.setCursor(0, 1);
      lcd.print("                ");

      break;

    default:

      if (opStateChanged) {

        lcd.setCursor(0, 1);
        lcd.print("T/A=");

        lcd.setCursor(7, 1);
        lcd.print("/");

        lcd.setCursor(13, 1);
        lcd.write(1);
        lcd.print("c");

        targetChanged = true;
        actualChanged = true;

      }

      if (targetChanged) {

        lcd.setCursor(4, 1);
        dtostrf(settings.manSettings.setPoint, 3, DISPLAY_TARGET_DECIMALS, lcd_buffer);
        lcd.print(lcd_buffer);

      }

      if (actualChanged) {

        lcd.setCursor(8, 1);
        dtostrf(actual, 5, DISPLAY_ACTUAL_DECIMALS, lcd_buffer);
        lcd.print(lcd_buffer);

      }

      break;

  }

  opStateChanged = false;
  selectedStateChanged = false;
  targetChanged = false;
  actualChanged = false;

}

void blinkLCD(uint8_t colour = RED) {

  if (lastBacklight != colour) {

    lcd.setBacklight(colour);
    lastBacklight = colour;

    if (colour == RED) {

      tone(BUZZER_WIRE_PWR, BUZZER_FREQUENCY);

    }

  }
  else {

    lcd.setBacklight(OFF);
    lastBacklight = OFF;
    noTone(BUZZER_WIRE_PWR);

  }

}

void displaySuccess(char message[]) {

  lcd.setCursor(0, 0);
  lcd.print(message);
  lcd.setCursor(0, 1);
  lcd.print("                ");

  blinkLCD(GREEN);
  delay(1000);
  lcd.setBacklight(WHITE);

}

void blinkRed() {

  blinkLCD(RED);

}

void displayError(char message[]) {

  lcd.setCursor(0, 0);
  lcd.print("     Error!     ");
  lcd.setCursor(0, 1);
  lcd.print(message);

  while (!readButtons()) {

    doFunctionAtInterval(blinkRed, &lastBlinkLCD, 250);

  }

  opState = OFF;
  lcd.setBacklight(WHITE);

}

void updateSettings() {

  EEPROM.updateBlock(configAddress, settings);

}
