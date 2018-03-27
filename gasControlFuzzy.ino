#include <OneWire.h>						// https://github.com/bigjosh/OneWireNoResistor
#include <DallasTemperature.h>					// https://github.com/milesburton/Arduino-Temperature-Control-Library

#include <FuzzyRule.h>						// https://github.com/zerokol/eFLL
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

#include <AccelStepper.h>					// https://github.com/adafruit/AccelStepper
#include <Adafruit_MotorShield.h>				// https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
#include "utility/Adafruit_MS_PWMServoDriver.h"

/*-----( Definitions )-----*/

#define ONE_WIRE_BUS 2						// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_PWR 3						// Use GPIO pins for power/ground to simplify the wiring
#define ONE_WIRE_GND 4						// Use GPIO pins for power/ground to simplify the wiring

#define SENSOR_PRECISION 12

#define READ_TEMP_SENSORS_EVERY 1000
#define READ_USER_INPUT_EVERY 20
#define COMPUTE_FUZZY_EVERY 2000

/*-----( Constants )-----*/

const int CONFIG_VERSION = 0.5;
const int memoryBase = 32;

const int jogSize = 16;

/*----( Objects )----*/

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

enum operatingState { OFF = 0, IGNITION, MAN, AUTO, SETUP };

struct gasDialSettings {
  int minPosition;
  int maxPosition;
};

struct allSettings {
  int version;
  gasDialSettings gasSettings;
  double setPoint;
};

// Place default values for settings here
allSettings settings = { CONFIG_VERSION, { 550, 1120 }, 55 };

/*----( Global Variables )----*/

// Operating State
operatingState opState = OFF;

// Serial input
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];// temporary array for use when parsing
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
double doubleFromPC = 0.0;
boolean newData = false;

// Logic
double actual = 50;
double error = -100;

// Processing
Fuzzy* fuzzy = new Fuzzy();

FuzzySet* errorN = new FuzzySet(-100, -100, -10, -5); // Negative Error
FuzzySet* errorZero = new FuzzySet(-15, -10, 0, 10); // Zero Error
FuzzySet* errorP = new FuzzySet(5, 10, 100, 100); // Positive Error

FuzzySet* errorChangeN = new FuzzySet(-5, -5, -0.1, -0.05); // Negative ErrorChange
FuzzySet* errorChangeZero = new FuzzySet(-0.1, 0, 0, 0.1); // Zero ErrorChange
FuzzySet* errorChangeP = new FuzzySet(0.05, 0.1, 5, 5); // Positive ErrorChange

FuzzySet* decrease = new FuzzySet(-70, -60, -60, -50); // decrease gas
FuzzySet* decreaseSmall = new FuzzySet(-45, -35, -35, -25); // small decrease gas
FuzzySet* zeroChange = new FuzzySet(-20, 0, 0, 10); // zero change
FuzzySet* increaseSmall = new FuzzySet(15, 25, 25, 35); // small increase gas
FuzzySet* increase = new FuzzySet(40, 50, 50, 60); // increase gas

// Loop Timing
unsigned long nextTemperatureRead = 0;
unsigned long nextUserInput = 0;
unsigned long nextFuzzyCompute = 0;


void setup() {

  Serial.begin(9600);
  Serial.println("Stubby's Brewduino!");

  initTempSensor();
  initFuzzyLogic();
  initStepper();

}

void doFunctionAtInterval(void (*callBackFunction)(), unsigned long *nextEvent, unsigned long interval) {

  unsigned long now = millis();

  if (now  >= *nextEvent)
  {
    *nextEvent = now + interval;
    callBackFunction();
  }

}

void loop() {

  doFunctionAtInterval(readUserInput, &nextUserInput, READ_USER_INPUT_EVERY);
  doFunctionAtInterval(readTempSensor, &nextTemperatureRead, READ_TEMP_SENSORS_EVERY);
  doFunctionAtInterval(processFuzzyLogic, &nextFuzzyCompute, COMPUTE_FUZZY_EVERY);
  stepper.run();

}

void initTempSensor() {

  sensors.begin();

  Serial.print("Locating temperature sensor...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);			// locate devices on the bus
  Serial.println(" devices.");

  if (!sensors.getAddress(tempSensor, 0)) {

    Serial.println("Unable to find address for Device 0");
    //displayError("   No sensor!    ");

  } else {

    sensors.setResolution(tempSensor, SENSOR_PRECISION);
    sensors.setWaitForConversion(false);  			// makes it async
    sensors.requestTemperatures();				// prime the pump for the next one - but don't wait

    Serial.println("Success!");

  }

}

void initFuzzyLogic() {

  FuzzyInput* error = new FuzzyInput(1);// With its ID in param

  error->addFuzzySet(errorN); // Add FuzzySet errorN to error
  error->addFuzzySet(errorZero); // Add FuzzySet errorZero to error
  error->addFuzzySet(errorP); // Add FuzzySet errorN to error

  fuzzy->addFuzzyInput(error);  // Add FuzzyInput to Fuzzy object

  FuzzyInput* errorChange = new FuzzyInput(2);// With its ID in param

  errorChange->addFuzzySet(errorChangeN);
  errorChange->addFuzzySet(errorChangeZero);
  errorChange->addFuzzySet(errorChangeP);

  fuzzy->addFuzzyInput(errorChange); // Add FuzzyInput to Fuzzy object

  FuzzyOutput* gasOutput = new FuzzyOutput(1);// With its ID in param

  gasOutput->addFuzzySet(decrease); // Add FuzzySet decrease to gasOuput
  gasOutput->addFuzzySet(decreaseSmall); // Add FuzzySet decreaseSmall to gasOuput
  gasOutput->addFuzzySet(zeroChange); // Add FuzzySet zeroChange to gasOuput
  gasOutput->addFuzzySet(increaseSmall); // Add FuzzySet increaseSmall to gasOuput
  gasOutput->addFuzzySet(increase); // Add FuzzySet decrease to gasOuput

  fuzzy->addFuzzyOutput(gasOutput); // Add FuzzyOutput to Fuzzy object

  FuzzyRuleAntecedent* ifErrorN = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorN->joinSingle(errorN); // Adding corresponding FuzzySet to Antecedent object

  FuzzyRuleAntecedent* ifErrorZero = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorZero->joinSingle(errorZero); // Adding corresponding FuzzySet to Antecedent object

  FuzzyRuleAntecedent* ifErrorP = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorP->joinSingle(errorP); // Adding corresponding FuzzySet to Antecedent object

  FuzzyRuleAntecedent* ifErrorZeroANDErrorChangeN = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorZeroANDErrorChangeN->joinWithAND(errorZero, errorChangeN);

  FuzzyRuleAntecedent* ifErrorZeroANDErrorChangeP = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorZeroANDErrorChangeP->joinWithAND(errorZero, errorChangeP);

  FuzzyRuleConsequent* thenGasOutputDecrease = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputDecrease->addOutput(decrease);// Adding corresponding FuzzySet to Consequent object

  FuzzyRuleConsequent* thenGasOutputDecreaseSmall = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputDecreaseSmall->addOutput(decreaseSmall);// Adding corresponding FuzzySet to Consequent object

  FuzzyRuleConsequent* thenGasOutputZeroChange = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputZeroChange->addOutput(zeroChange);// Adding corresponding FuzzySet to Consequent object

  FuzzyRuleConsequent* thenGasOutputIncreaseSmall = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputIncreaseSmall->addOutput(increaseSmall);// Adding corresponding FuzzySet to Consequent object

  FuzzyRuleConsequent* thenGasOutputIncrease = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputIncrease->addOutput(increase);// Adding corresponding FuzzySet to Consequent object


  // FuzzyRule "IF error is Zero THEN gasOutput is zeroChange"
  FuzzyRule* fuzzyRule01 = new FuzzyRule(1, ifErrorZero, thenGasOutputZeroChange); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule(fuzzyRule01); // Adding FuzzyRule to Fuzzy object

  // FuzzyRule "IF error is negative THEN gasOutput is increase"
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2, ifErrorN, thenGasOutputIncrease); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule(fuzzyRule02); // Adding FuzzyRule to Fuzzy object

  // FuzzyRule "IF error is positive THEN gasOutput is decrease"
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3, ifErrorP, thenGasOutputDecrease); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule(fuzzyRule03); // Adding FuzzyRule to Fuzzy object

  // Fuzzyrule "IF error is Zero AND error change is positive THEN gasOutput decreaseSmall
  FuzzyRule* fuzzyRule04 = new FuzzyRule(4, ifErrorZeroANDErrorChangeP, thenGasOutputDecreaseSmall); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule(fuzzyRule04); // Adding FuzzyRule to Fuzzy object

  // Fuzzyrule "IF error is Zero AND error change is negative THEN gasOutput increaseSmall
  FuzzyRule* fuzzyRule05 = new FuzzyRule(5, ifErrorZeroANDErrorChangeN, thenGasOutputIncreaseSmall); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule(fuzzyRule05); // Adding FuzzyRule to Fuzzy object

}

void initStepper() {

  AFMS.begin(); // Start the bottom shield

  stepper.setMaxSpeed(200.0);
  stepper.setAcceleration(100.0);
  stepper.setCurrentPosition(0);

}

void readTempSensor() {

  actual = double(sensors.getTempC(tempSensor));
  sensors.requestTemperatures(); 				// prime the pump for the next one - but don't wait

}

void readUserInput() {
  readSerialInput();
}

void readSerialInput() {

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
    //   because strtok() used in parseData() replaces the commas with \0
    parseSerialInput();
    newData = false;
  }

}

void parseSerialInput() {

  // split the data into its parts
  char * strtokIndx;	// this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");	// get the first part - the string
  strcpy(messageFromPC, strtokIndx);	// copy it to messageFromPC
  strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
  doubleFromPC = atof(strtokIndx);

  switch (opState) {

    case 2:		//MANUAL
      if (String(messageFromPC) == "L") {
        Serial.print("Moving to "); Serial.println(stepper.currentPosition() + doubleFromPC);
        stepper.moveTo(stepper.currentPosition() + doubleFromPC);
        return;
      }

      if (String(messageFromPC) == "R") {
        Serial.print("Moving to "); Serial.println(stepper.currentPosition() - doubleFromPC);
        stepper.moveTo(stepper.currentPosition() - doubleFromPC);
        return;
      }
      if (String(messageFromPC) == "DIAL") {
        Serial.print("Moving to "); Serial.println(doubleFromPC);
        stepper.moveTo(doubleFromPC);
        return;
      }

    case 3:		//AUTO
      if (String(messageFromPC) == "SETPOINT") {
        Serial.print("Setting target temperature to "); Serial.println(doubleFromPC);
        //settings.setPoint = doubleFromPC;
        //targetChanged = true;
        return;
      }

    case 4:		//SETUP
      if (String(messageFromPC) == "MAX") {
        Serial.print("Setting maximum dial position to "); Serial.println(doubleFromPC);
        //settings.gasSettings.maxPosition = stepper.currentPosition();
        //updateSettings();
        return;
      }

      if (String(messageFromPC) == "MIN") {
        Serial.print("Setting minimum dial position to "); Serial.println(doubleFromPC);
        //settings.gasSettings.minPosition = stepper.currentPosition();
        //updateSettings();
        return;
      }

    default:
      if (String(messageFromPC) == "MODE") {

        // enum operatingState { OFF = 0, IGNITION, MAN, AUTO, SETUP };

        Serial.print("Mode change ");
        // strtokIndx = strtok(NULL, ",");	// this continues where the previous call left off
        //doubleFromPC = atof(strtokIndx);

        switch ((int)doubleFromPC) {
          case 0:
            Serial.println("OFF");
            opState = OFF;
            Serial.print("Moving to "); Serial.println(0);
            stepper.moveTo(0);
            break;

          case 1:
            Serial.println("IGNITION");
            opState = IGNITION;
            Serial.println("Turning on gas now! IGNITE!");
            stepper.runToNewPosition(1120);
            delay(5000);
            Serial.println("Mode change MANUAL");
            opState = MAN;
            Serial.print("Moving to "); Serial.println(550);
            stepper.moveTo(550);
            break;

          case 2:
            Serial.println("MANUAL");
            opState = MAN;
            break;

          case 3:
            Serial.println("AUTO");
            opState = AUTO;
            break;

          case 4:
            Serial.println("SETUP");
            opState = SETUP;
            break;

          default:
            Serial.println("not recognised!");
        }

        return;
      }
      Serial.println ("Command not recognised in this mode.");

  }

}

void processFuzzyLogic() {

  if (opState == AUTO) {

    double previousError = error;

    //actual = actual + random(-1, 1);
    Serial.print("Actual: "); Serial.println(actual, 2);
    error = actual - 50;
    fuzzy->setInput(1, error);

    fuzzy->setInput(2, (error - previousError) / (COMPUTE_FUZZY_EVERY / 1000.0));

    fuzzy->fuzzify();
    /*
      Serial.print("error: ");
      Serial.print(errorN->getPertinence());
      Serial.print(", ");
      Serial.print(errorZero->getPertinence());
      Serial.print(", ");
      Serial.println(errorP->getPertinence());

      Serial.print("errorChange: ");
      Serial.print(errorChangeN->getPertinence());
      Serial.print(", ");
      Serial.print(errorChangeZero->getPertinence());
      Serial.print(", ");
      Serial.println(errorChangeP->getPertinence());

      Serial.print("gasOutput: ");
      Serial.print(decrease->getPertinence());
      Serial.print(", ");
      Serial.print(decreaseSmall->getPertinence());
      Serial.print(", ");
      Serial.print(zeroChange->getPertinence());
      Serial.print(", ");
      Serial.print(increaseSmall->getPertinence());
      Serial.print(", ");
      Serial.println(increase->getPertinence());

      bool wasTheRuleFired;

      for (int i = 1; i <= 5; i++) {
        wasTheRuleFired = fuzzy->isFiredRule(i);
        if (wasTheRuleFired) Serial.println(i);
      }
    */
    float output = fuzzy->defuzzify(1);

    //Serial.print("Output: "); Serial.println(output, 2);
    int tPosition = (int)(stepper.currentPosition() + ((output / 100) * (1120 - 550)));
    tPosition = min(1120, tPosition);
    tPosition = max(550, tPosition);
    stepper.moveTo(tPosition);
    Serial.print("tPosition: "); Serial.println(tPosition);

  }

}


