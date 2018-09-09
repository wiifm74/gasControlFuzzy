const int CONFIG_VERSION = 0;

#define FEATURE_ENABLED_MYSETTINGS

#include <EEPROMex.h>                               // https://github.com/thijse/Arduino-EEPROMEx
#include <avr/pgmspace.h>

#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>                  // https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library
#include <utility/Adafruit_MCP23017.h>

#include <OneWire.h>                                // https://github.com/bigjosh/OneWireNoResistor
#include <DallasTemperature.h>                      // https://github.com/milesburton/Arduino-Temperature-Control-Library

#include <FuzzyRule.h>                              // https://github.com/zerokol/eFLL
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

#include <AccelStepper.h>                           // https://github.com/adafruit/AccelStepper
#include <Adafruit_MotorShield.h>                   // https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
#include "utility/Adafruit_MS_PWMServoDriver.h"

/*-----( Definitions )-----*/

#include "Definitions.h"

/*-----( Constants )-----*/

const int memoryBase = 32;

const char opState0[] PROGMEM = "OFF";
const char opState1[] PROGMEM = "IGNITION";
const char opState2[] PROGMEM = "AUTOMATIC";
const char opState3[] PROGMEM = "MANUAL";
const char opState4[] PROGMEM = "PUMP";
const char* const opStateTable[] PROGMEM = {opState0, opState1, opState2, opState3, opState4};

const int jogSize = 32;

/*----( Objects )----*/

OneWire oneWire( ONE_WIRE_BUS );                                // Create a oneWire instance to communicate with any OneWire devices ( not just Maxim/Dallas temperature ICs )
DallasTemperature sensors( &oneWire );                          // Pass our oneWire reference to Dallas Temperature.
DeviceAddress tempSensor;                                       // Arrays to hold device address

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();            // Create a RGB LCD instance

Adafruit_MotorShield AFMS = Adafruit_MotorShield();             // Create a motor shield object with the default I2C address
Adafruit_StepperMotor *myStepper = AFMS.getStepper( 200, 2 );   // Connect a stepper motor with 200 steps per revolution ( 1.8 degree ) to motor port #2 ( M3 and M4 )

void forwardstep() {
  myStepper->onestep( FORWARD, MICROSTEP );           // Anti-clockwise
}

void backwardstep() {
  myStepper->onestep( BACKWARD, MICROSTEP );           // Clockwise
}

AccelStepper stepper( forwardstep, backwardstep );     // Wrap the stepper in an AccelStepper object

enum operatingState { OFF = 0, IGNITION, AUTO, MAN, PUMP };
enum pumpStates { NORECIRC = 0, RECIRC };
enum autoDisplay { RECIPETITLE = 0, STEPTITLE, TA, TIMEREMAINING };

struct sensorCalibration {
  double rawLow;
  double rawHigh;
};

struct recipeStep {
  bool active;
  char title[16];
  char startMessage[16];
  char endMessage[16];
  double temperature;
  int stepTime;
};

struct recipe {
  char title[16];
  recipeStep steps[6];
  int hopsAddition[3];
};

struct allSettings {
  int version;
  sensorCalibration tempCal;
  double minPosition;
  double maxPosition;
  int currentRecipe;
  int currentStep;
  int currentHopsAddition;
};

/*----( Global Variables )----*/

// EEPROM
int configAddress;

//#include "Recipes.h"
#include "SousVide.h"
//#include "TestingRecipes.h"

// Settings
#ifdef FEATURE_ENABLED_MYSETTINGS
#include "mySettings.h"
#else
allSettings settings = { CONFIG_VERSION, { 0.0, 100.0 }, 0, 0, -1, 0, 0 };
#endif


// Operating State
operatingState opState = OFF;
boolean opStateChanged = true;
bool ignitionHasOccured = false;
bool userResponseWaterFilled = false;

// Pump
pumpStates pumpState = NORECIRC;
boolean pumpStateChanged = true;

// User Input
uint8_t lastButtonPressed = 0;
operatingState selectedState = OFF;
boolean selectedStateChanged = true;

// Communications
char receivedChars[BUFFER_SIZE];
char tempChars[BUFFER_SIZE];// temporary array for use when parsing
char bufferChars[BUFFER_SIZE] = {0};
char messageToPC[BUFFER_SIZE] = {0};
double doubleFromPC = 0.0;
boolean newData = false;

// LCD
bool targetChanged = true;
bool actualChanged = true;
bool autoDisplayChanged = true;
autoDisplay autoDisplayState = TA;
uint8_t lastBacklight = OFF;

byte degree[] =            // define the degree symbol
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

// Logic
double actual = 0;
double error = 0;

// Auto mode
unsigned long stepTargetReached = 0;

// Processing
Fuzzy* fuzzy = new Fuzzy();
#include "FuzzySets.h"

// Loop Timing
unsigned long nextTemperatureRead = 0;
unsigned long nextUserInput = 0;
unsigned long nextAutoCompute = 0;
unsigned long nextDisplayWrite = 0;
unsigned long nextBlinkLCD = 0;
unsigned long nextAutoDisplayChange = 0;
unsigned long nextPumpUpdate = 0;

/*----( Functions )----*/

void setup() {
  Serial.begin( 9600 );
  while ( !Serial ); // wait for serial port to connect. Needed for native USB
  Serial.println( "Fuzzy Gas Brewduino!" );
  initBuzzer();
  initPump();
  initSettings();
  initTempSensor();
  initDisplay();
  initFuzzyLogic();
  initStepper();
}

void doFunctionAtInterval( void ( *callBackFunction )(), unsigned long *nextEvent, unsigned long interval ) {
  unsigned long now = millis();
  if ( now  >= *nextEvent ) {
    *nextEvent = now + interval;
    callBackFunction();
  }
}

void loop() {
  doFunctionAtInterval( readUserInput, &nextUserInput, READ_USER_INPUT_EVERY );
  doFunctionAtInterval( readTempSensor, &nextTemperatureRead, READ_TEMP_SENSORS_EVERY );
  doFunctionAtInterval( processAutomaticMode, &nextAutoCompute, COMPUTE_AUTO_EVERY );
  doFunctionAtInterval( updateDisplay, &nextDisplayWrite, WRITE_DISPLAY_EVERY );
  doFunctionAtInterval( iterateAutoDisplay, &nextAutoDisplayChange, CHANGE_AUTO_EVERY );
  doFunctionAtInterval( updatePump, &nextPumpUpdate, UPDATE_PUMP_EVERY );
  stepper.run();
}

void initSettings() {
  allSettings tempSettings;
  int timeItTook = 0;
  EEPROM.setMemPool( memoryBase, EEPROMSizeMega );
  configAddress = EEPROM.getAddress( sizeof( allSettings ) );
  timeItTook = EEPROM.readBlock( configAddress, tempSettings ); // Read EEPROM settings to temporary location to compare CONFIG_VERSION
  // Update EEPROM from new settings configuration if necessary
  if ( tempSettings.version != CONFIG_VERSION ) {
    timeItTook = EEPROM.writeBlock( configAddress, settings );  // Settings have not been saved before or settings configuration has changed
    Serial.println( "Uploading new settings to EEPROM" );
  }
  timeItTook = EEPROM.readBlock( configAddress, settings ); // Read settings from EEPROM
}

void updateSettings() {
  EEPROM.updateBlock( configAddress, settings );
}

void initTempSensor() {
  sensors.begin();
  Serial.print( F( "Locating temperature sensor... " ) );
  Serial.print( F( "Found " ) );
  Serial.print( sensors.getDeviceCount(), DEC );      // locate devices on the bus
  Serial.println( F( " device( s )." ) );
  if ( !sensors.getAddress( tempSensor, 0 ) ) {
    Serial.println( F( "Unable to find address for Device 0" ) );
    displayError( "No sensor!" );
  } else {
    sensors.setResolution( tempSensor, SENSOR_PRECISION );
    sensors.setWaitForConversion( false );        // makes it async
    sensors.requestTemperatures();        // prime the pump for the next one - but don't wait
  }
}

void readTempSensor() {
  double previous = actual;
  actual = ( ( ( double( sensors.getTempC( tempSensor ) ) - settings.tempCal.rawLow ) * ( 100 - 0 ) ) / ( settings.tempCal.rawHigh - settings.tempCal.rawLow ) ) + 0;
  if ( previous != actual ) {
    actualChanged = true;
  }
  sensors.requestTemperatures();        // prime the pump for the next one - but don't wait
}

void initFuzzyLogic() {
  FuzzyInput* error = new FuzzyInput( 1 );// With its ID in param
  error->addFuzzySet( errorN ); // Add FuzzySet errorN to error
  error->addFuzzySet( errorZero ); // Add FuzzySet errorZero to error
  error->addFuzzySet( errorP ); // Add FuzzySet errorN to error
  fuzzy->addFuzzyInput( error );  // Add FuzzyInput to Fuzzy object
  FuzzyInput* errorChange = new FuzzyInput( 2 );// With its ID in param
  errorChange->addFuzzySet( errorChangeN );
  errorChange->addFuzzySet( errorChangeZero );
  errorChange->addFuzzySet( errorChangeP );
  fuzzy->addFuzzyInput( errorChange ); // Add FuzzyInput to Fuzzy object
  FuzzyOutput* gasOutput = new FuzzyOutput( 1 );// With its ID in param
  gasOutput->addFuzzySet( decrease ); // Add FuzzySet decrease to gasOuput
  gasOutput->addFuzzySet( decreaseSmall ); // Add FuzzySet decreaseSmall to gasOuput
  gasOutput->addFuzzySet( zeroChange ); // Add FuzzySet zeroChange to gasOuput
  gasOutput->addFuzzySet( increaseSmall ); // Add FuzzySet increaseSmall to gasOuput
  gasOutput->addFuzzySet( increase ); // Add FuzzySet decrease to gasOuput
  fuzzy->addFuzzyOutput( gasOutput ); // Add FuzzyOutput to Fuzzy object
  FuzzyRuleAntecedent* ifErrorN = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorN->joinSingle( errorN ); // Adding corresponding FuzzySet to Antecedent object
  FuzzyRuleAntecedent* ifErrorZero = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorZero->joinSingle( errorZero ); // Adding corresponding FuzzySet to Antecedent object
  FuzzyRuleAntecedent* ifErrorP = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorP->joinSingle( errorP ); // Adding corresponding FuzzySet to Antecedent object
  FuzzyRuleAntecedent* ifErrorZeroANDErrorChangeN = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorZeroANDErrorChangeN->joinWithAND( errorZero, errorChangeN );
  FuzzyRuleAntecedent* ifErrorZeroANDErrorChangeP = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifErrorZeroANDErrorChangeP->joinWithAND( errorZero, errorChangeP );
  FuzzyRuleConsequent* thenGasOutputDecrease = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputDecrease->addOutput( decrease );// Adding corresponding FuzzySet to Consequent object
  FuzzyRuleConsequent* thenGasOutputDecreaseSmall = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputDecreaseSmall->addOutput( decreaseSmall );// Adding corresponding FuzzySet to Consequent object
  FuzzyRuleConsequent* thenGasOutputZeroChange = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputZeroChange->addOutput( zeroChange );// Adding corresponding FuzzySet to Consequent object
  FuzzyRuleConsequent* thenGasOutputIncreaseSmall = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputIncreaseSmall->addOutput( increaseSmall );// Adding corresponding FuzzySet to Consequent object
  FuzzyRuleConsequent* thenGasOutputIncrease = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenGasOutputIncrease->addOutput( increase );// Adding corresponding FuzzySet to Consequent object
  // FuzzyRule "IF error is Zero THEN gasOutput is zeroChange"
  FuzzyRule* fuzzyRule01 = new FuzzyRule( 1, ifErrorZero, thenGasOutputZeroChange ); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule( fuzzyRule01 ); // Adding FuzzyRule to Fuzzy object
  // FuzzyRule "IF error is negative THEN gasOutput is increase"
  FuzzyRule* fuzzyRule02 = new FuzzyRule( 2, ifErrorN, thenGasOutputIncrease ); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule( fuzzyRule02 ); // Adding FuzzyRule to Fuzzy object
  // FuzzyRule "IF error is positive THEN gasOutput is decrease"
  FuzzyRule* fuzzyRule03 = new FuzzyRule( 3, ifErrorP, thenGasOutputDecrease ); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule( fuzzyRule03 ); // Adding FuzzyRule to Fuzzy object
  // Fuzzyrule "IF error is Zero AND error change is positive THEN gasOutput decreaseSmall
  FuzzyRule* fuzzyRule04 = new FuzzyRule( 4, ifErrorZeroANDErrorChangeP, thenGasOutputDecreaseSmall ); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule( fuzzyRule04 ); // Adding FuzzyRule to Fuzzy object
  // Fuzzyrule "IF error is Zero AND error change is negative THEN gasOutput increaseSmall
  FuzzyRule* fuzzyRule05 = new FuzzyRule( 5, ifErrorZeroANDErrorChangeN, thenGasOutputIncreaseSmall ); // Passing the Antecedent and the Consequent of expression
  fuzzy->addFuzzyRule( fuzzyRule05 ); // Adding FuzzyRule to Fuzzy object
}

void processAutomaticMode() {
  if ( opState == AUTO ) {
    double previousError = error;
    error = actual - recipes[settings.currentRecipe].steps[settings.currentStep].temperature;
    if ( !stepTargetReached && ( error >= 0 ) ) {
      // target temperature has been reached. start countdown
      stepTargetReached = millis();
    }
    // hops additions
    if ( stepTargetReached && settings.currentStep == 5 ) {
      if ( settings.currentHopsAddition <= 2 ) {
        if ( stepTargetReached + recipes[settings.currentRecipe].steps[settings.currentStep].stepTime * MIN_TO_MS - millis() <= recipes[settings.currentRecipe].hopsAddition[settings.currentHopsAddition] * MIN_TO_MS ) {
          displayAlert( "ADD HOPS" );
          settings.currentHopsAddition++;
        }
      }
    }
    if ( stepTargetReached && ( stepTargetReached + ( recipes[settings.currentRecipe].steps[settings.currentStep].stepTime * MIN_TO_MS ) <= millis() ) ) {
      // end of step has been reached
      // last hops
      if ( recipes[settings.currentRecipe].hopsAddition[settings.currentHopsAddition] == 0 ) {
        displayAlert( "ADD HOPS" );
      }
      // perform end user alert
      if ( String( recipes[settings.currentRecipe].steps[settings.currentStep].endMessage ) != "" ) {
        displayAlert( recipes[settings.currentRecipe].steps[settings.currentStep].endMessage );
      }
      // increment step
      while ( true ) {
        settings.currentStep++;
        if ( settings.currentStep > 5 ) {
          // Automatic mode is complete
          settings.currentRecipe = -1;
          settings.currentStep = 0;
          settings.currentHopsAddition = 0;
          opState = OFF;
          processModeChange();
          updateSettings();
          return;
        }
        if ( recipes[settings.currentRecipe].steps[settings.currentStep].active ) {
          // start of next step
          stepTargetReached = 0;
          pumpState = ( settings.currentStep < 5 ? RECIRC : NORECIRC );
          pumpStateChanged = true;
          // perform start user alert
          if ( String( recipes[settings.currentRecipe].steps[settings.currentStep].startMessage ) != "" ) {
            displayAlert( recipes[settings.currentRecipe].steps[settings.currentStep].startMessage );
          }
          updateSettings();
          break;
        }
      }
    }
    fuzzy->setInput( 1, error );
    fuzzy->setInput( 2, ( error - previousError ) / ( COMPUTE_AUTO_EVERY / 1000.0 ) );
    fuzzy->fuzzify();
    float output = fuzzy->defuzzify( 1 );
    int tPosition = ( int )( stepper.currentPosition() + ( ( output / 100 ) * ( settings.maxPosition - settings.minPosition ) ) );
    tPosition = min( settings.maxPosition, tPosition );
    tPosition = max( settings.minPosition, tPosition );
    stepper.moveTo( ( settings.currentStep == 5 ? settings.maxPosition : tPosition ) );
  }
}

void initDisplay() {
  lcd.begin( 16, 2 );
  lcd.clear();
  lcd.createChar( 1, degree );          // create degree symbol from the binary
  lcd.setBacklight( WHITE );
  lcd.print( centreForLCD( 16,  "Fuzzy Gas" ) );
  lcd.setCursor( 0, 1 );
  lcd.print( centreForLCD( 16,  "Brewduino!" ) );
  nextDisplayWrite = millis() + 2000;       // Splash screen delay
}

void updateDisplay() {
  // Top line of LCD
  if ( opStateChanged  ) {
    strcpy_P( messageToPC, ( char* )pgm_read_word( &( opStateTable[opState] ) ) );
    lcd.setCursor( 0, 0 );
    lcd.print( ( opState == OFF ? " " : "<" ) + centreForLCD( 14, String( messageToPC ) ) + ( opState == PUMP ? " " : ">" ) );
  }
  if ( selectedStateChanged ) {
    strcpy_P( messageToPC, ( char* )pgm_read_word( &( opStateTable[selectedState] ) ) );
    lcd.setCursor( 0, 0 );
    lcd.print( ( selectedState == OFF ? " " : "<" ) + centreForLCD( 14, String( messageToPC ) ) + ( selectedState == PUMP ? " " : ">" ) );
  }
  String bottomLine = "";
  bool degCRequired = false;
  // Create Message
  if ( opStateChanged || actualChanged || targetChanged || autoDisplayChanged || pumpStateChanged ) {
    switch ( opState ) {
      case OFF:
        degCRequired = true;
        dtostrf( actual, 0, DISPLAY_ACTUAL_DECIMALS, messageToPC );
        bottomLine = "A=" + String( messageToPC );
        break;
      case IGNITION:
        bottomLine = "IGNITE!";
        break;
      case AUTO:
        switch ( autoDisplayState ) {
          case TA:
            degCRequired = true;
            dtostrf( recipes[settings.currentRecipe].steps[settings.currentStep].temperature, 0, DISPLAY_ACTUAL_DECIMALS, messageToPC );
            bottomLine = "T/A=" + String( messageToPC ) + "/";
            dtostrf( actual, 0, DISPLAY_ACTUAL_DECIMALS, messageToPC );
            // bottomLine = "T/A=" + String( recipes[settings.currentRecipe].steps[settings.currentStep].temperature ) + "/" + String( messageToPC );
            bottomLine += String( messageToPC );
            break;
          case RECIPETITLE:
            bottomLine = recipes[settings.currentRecipe].title;
            break;
          case STEPTITLE:
            bottomLine = recipes[settings.currentRecipe].steps[settings.currentStep].title;
            break;
          case TIMEREMAINING:
            if ( stepTargetReached ) {
              bottomLine = msToHMS( stepTargetReached + ( recipes[settings.currentRecipe].steps[settings.currentStep].stepTime * MIN_TO_MS ) <= millis() ? 0 : ( stepTargetReached + ( recipes[settings.currentRecipe].steps[settings.currentStep].stepTime * MIN_TO_MS ) ) - millis() );
            } else {
              bottomLine = msToHMS( recipes[settings.currentRecipe].steps[settings.currentStep].stepTime * MIN_TO_MS );
            }
            break;
          default:
            break;
        }
        break;
      case MAN:
        degCRequired = true;
        dtostrf( actual, 0, DISPLAY_ACTUAL_DECIMALS, messageToPC );
        bottomLine = "D/A=" + String( stepper.currentPosition() ) + "/" + String( messageToPC );
        break;
      case PUMP:
        bottomLine = ( pumpState ? "ON" : "OFF" );
        break;
      default:
        break;
    }
    // Write message including leading spaces
    bottomLine = addLeadingSpacesForCentering( 16 - ( degCRequired ? 2 : 0 ), bottomLine );
    lcd.setCursor( 0, 1 );
    lcd.print( bottomLine );
    // Write degrees c if necessary
    if ( degCRequired ) {
      lcd.write( 1 );
      lcd.print( "c" );
    } else {
    }
    bottomLine = addTrailingSpacesForCentering( 16 - bottomLine.length() - ( degCRequired ? 2 : 0 ), "" );
    // Write trailing space
    lcd.print( bottomLine );
  }
  opStateChanged = false;
  selectedStateChanged = false;
  actualChanged = false;
  targetChanged = false;
  autoDisplayChanged = false;
  pumpStateChanged = false;
}

void blinkLCD( uint8_t colour = RED ) {
  if ( lastBacklight != colour ) {
    lcd.setBacklight( colour );
    lastBacklight = colour;
    if ( colour != WHITE) {
      tone( BUZZER_WIRE_PWR, BUZZER_FREQUENCY );
    }
  }
  else {
    lcd.setBacklight( OFF );
    lastBacklight = OFF;
    noTone( BUZZER_WIRE_PWR );
  }
}

void displaySuccess( const char message[] ) {
  lcd.setCursor( 0, 0 );
  lcd.print( message );
  lcd.setCursor( 0, 1 );
  lcd.print( "                " );
  blinkLCD( GREEN );
  delay( 1000 );
  lcd.setBacklight( WHITE );
}

void blinkRed() {
  blinkLCD( RED );
}

void blinkBlue() {
  blinkLCD( BLUE );
}

void displayAlert( const char message[] ) {
  // remember previous dial position, move dial to minimum position
  int pPosition = stepper.currentPosition();
  stepper.moveTo( settings.minPosition );
  // top Line
  lcd.setCursor( 0, 0 );
  lcd.print( centreForLCD( 16, "ALERT!" ) );
  // bottom line
  lcd.setCursor( 0, 1 );
  lcd.print( centreForLCD( 16, message ) );
  // wait for user response
  while ( !readButtons() ) {
    doFunctionAtInterval( blinkBlue, &nextBlinkLCD, 250 );
  }
  noTone( BUZZER_WIRE_PWR );
  // put dial in previous position
  stepper.moveTo( pPosition );
  lcd.setBacklight( WHITE );
  opStateChanged = true;
  actualChanged = true;
}

void displayError( const char message[] ) {
  lcd.setCursor( 0, 0 );
  lcd.print( centreForLCD( 16, "ERROR!" ) );
  lcd.setCursor( 0, 1 );
  lcd.print( message );
  while ( !readButtons() ) {
    doFunctionAtInterval( blinkRed, &nextBlinkLCD, 250 );
  }
  noTone( BUZZER_WIRE_PWR );
  lcd.setBacklight( WHITE );
  opState = OFF;
  processModeChange();
}

String centreForLCD( int lineLength, String str ) {
  return addTrailingSpacesForCentering( lineLength, addLeadingSpacesForCentering( lineLength, str ) );
}

String addLeadingSpacesForCentering( int lineLength, String str ) {
  String result = "";
  for ( int i = 0; i < ( int )floor( ( double )( lineLength - str.length() ) / 2 ); i++ ) {
    result.concat( " " );
  }
  result.concat( str );
  return result;
}

String addTrailingSpacesForCentering( int lineLength, String str ) {
  String result = str;
  // Assumes leading spaces have been added already
  for ( int i = 0; i < ( lineLength - str.length() ); i++ ) {
    result.concat( " " );
  }
  return result;
}

bool getResponseYN( const char message[] ) {
  bool response = true;;
  uint8_t buttons;
  String bottomLine = "";
  lcd.setCursor( 0, 0 );
  lcd.print( centreForLCD( 16, message ) );
  lcd.setCursor( 0, 1 );
  lcd.print( "<" + centreForLCD( 14, String( ( response ? "YES" : "NO" ) ) ) + ">" );
  while (true) {
    buttons = readButtons();
    if ( buttons ) {
      if ( buttons & BUTTON_LEFT ) {
        response = !response;
      }
      if ( buttons & BUTTON_RIGHT ) {
        response = !response;
      }
      if ( buttons & BUTTON_SELECT ) {
        return response;
      }
      lcd.setCursor( 0, 1 );
      lcd.print( "<" + centreForLCD( 14, String( ( response ? "YES" : "NO" ) ) ) + ">" );
    }
  }
}

int getResponseChoices( const char message[], const char choices[][16], int numChoices ) {
  int response = 0;
  uint8_t buttons;
  String bottomLine = "";
  lcd.setCursor( 0, 0 );
  lcd.print( centreForLCD( 16, message ) );
  lcd.setCursor( 0, 1 );
  lcd.print( ( response == 0 ? " " : "<" ) + centreForLCD( 14, String( choices[response] ) ) + ( response == numChoices - 1 ? " " : ">" ) );
  while (true) {
    buttons = readButtons();
    if ( buttons ) {
      if ( buttons & BUTTON_LEFT ) {
        response = max( 0, response - 1 );
      }
      if ( buttons & BUTTON_RIGHT ) {
        response = min ( numChoices - 1, response + 1 );
      }
      if ( buttons & BUTTON_SELECT ) {
        return response;
      }
      lcd.setCursor( 0, 1 );
      lcd.print( ( response == 0 ? " " : "<" ) + centreForLCD( 14, String( choices[response] ) ) + ( response == numChoices - 1 ? " " : ">" ) );
    }
  }
}

void iterateAutoDisplay() {  
  autoDisplayState = ( autoDisplay )( autoDisplayState == ( recipes[settings.currentRecipe].steps[settings.currentStep].stepTime == 0  ? TA : TIMEREMAINING ) ? 0 : autoDisplayState + 1 );
  autoDisplayChanged = true;
}

String msToHMS( unsigned long ms ) {
  // 1- Convert to seconds:
  int seconds = ms / 1000;
  // 2- Extract hours:
  int hours = seconds / 3600 ;
  seconds = seconds % 3600; // seconds remaining after extracting hours
  // 3- Extract minutes:
  int minutes = seconds / 60; // 60 seconds in 1 minute
  // 4- Keep only seconds not extracted to minutes:
  seconds = seconds % 60;
  return String( hours ) + ( minutes < 10 ? ":0" : ":" ) + String( minutes ) + ( seconds < 10 ? ":0" : ":" ) + String( seconds );
}

void initStepper() {
  AFMS.begin(); // Start the bottom shield
  stepper.setMaxSpeed( 200.0 );
  stepper.setAcceleration( 100.0 );
  stepper.setCurrentPosition( 0 );
}

void readUserInput() {
  readButtonInput();
  readSerialInput();
}

void readButtonInput() {
  uint8_t buttons = readButtons();
  if ( buttons ) {
    if ( buttons & BUTTON_LEFT ) {
      selectedState = ( operatingState )max( OFF, int( selectedState ) - 1 );
      selectedStateChanged = true;
    }
    if ( buttons & BUTTON_RIGHT ) {
      selectedState = ( operatingState )min( PUMP, int( selectedState ) + 1 );
      selectedStateChanged = true;
    }
    if ( buttons & BUTTON_UP ) {
      switch ( opState ) {
        case MAN:
          stepper.moveTo( stepper.currentPosition() + jogSize );
          break;
        default:
          break;
      }
    }
    if ( buttons & BUTTON_DOWN ) {
      switch ( opState ) {
        case MAN:
          stepper.moveTo( stepper.currentPosition() - jogSize );
          break;
        default:
          break;
      }
    }
    if ( buttons & BUTTON_SELECT ) {
      opState = selectedState;
      processModeChange();
    }
  }
}

uint8_t readButtons() {
  uint8_t buttons = lcd.readButtons();
  if ( buttons ) {
    tone( BUZZER_WIRE_PWR, BUZZER_FREQUENCY );
    delay( 20 );
    noTone( BUZZER_WIRE_PWR );
    lastButtonPressed = buttons;
    return 0;              // Wait until button is release before sending
  }
  else {
    buttons = lastButtonPressed;
    lastButtonPressed = 0;
    return buttons;           // Button is released - send!
  }
}

void readSerialInput() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while ( Serial.available() > 0 && newData == false ) {
    rc = Serial.read();
    if ( recvInProgress == true ) {
      if ( rc != endMarker ) {
        receivedChars[ndx] = rc;
        ndx++;
        if ( ndx >= BUFFER_SIZE ) {
          ndx = BUFFER_SIZE - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';// terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if ( rc == startMarker ) {
      recvInProgress = true;
    }
  }
  if ( newData == true ) {
    strcpy( tempChars, receivedChars );
    // this temporary copy is necessary to protect the original data
    // because strtok() used in parseData() replaces the commas with \0
    parseSerialInput();
    newData = false;
  }
}

void parseSerialInput() {
  // split the data into its parts
  char * strtokIndx;  // this is used by strtok() as an index
  strtokIndx = strtok( tempChars, ", " );  // get the first part - the string
  strcpy( bufferChars, strtokIndx );  // copy it to bufferChars
  strtokIndx = strtok( NULL, ", " ); // this continues where the previous call left off
  doubleFromPC = atof( strtokIndx );
  Serial.print( bufferChars ); Serial.print( ": " );
  if ( String( bufferChars ) == "L" ) {
    Serial.println( stepper.currentPosition() + doubleFromPC );
    stepper.moveTo( stepper.currentPosition() + doubleFromPC );
    return;
  }
  if ( String( bufferChars ) == "R" ) {
    Serial.println( stepper.currentPosition() - doubleFromPC );
    stepper.moveTo( stepper.currentPosition() - doubleFromPC );
    return;
  }
  if ( String( bufferChars ) == "DIAL" ) {
    Serial.println( doubleFromPC );
    stepper.moveTo( doubleFromPC );
    return;
  }
  if ( String( bufferChars ) == "MAX" ) {
    Serial.println( doubleFromPC );
    settings.maxPosition = doubleFromPC;
    updateSettings();
    return;
  }
  if ( String( bufferChars ) == "MIN" ) {
    Serial.println( doubleFromPC );
    settings.minPosition = doubleFromPC;
    updateSettings();
    return;
  }
  if ( String( bufferChars ) == "RAWLOW" ) {
    Serial.println( doubleFromPC );
    settings.tempCal.rawLow = doubleFromPC;
    updateSettings();
    return;
  }
  if ( String( bufferChars ) == "RAWHIGH" ) {
    Serial.println( doubleFromPC );
    settings.tempCal.rawHigh = doubleFromPC;
    updateSettings();
    return;
  }
  if ( String( bufferChars ) == "MODE" ) {
    operatingState newOpState = ( operatingState )doubleFromPC;
    if ( newOpState == opState ) {
      Serial.println( "not required" );
      return;
    }
    if ( ( newOpState >= OFF ) && ( newOpState <= PUMP ) ) {
      strcpy_P( messageToPC, ( char* )pgm_read_word( &( opStateTable[newOpState] ) ) );
      Serial.println( messageToPC );
      opState = newOpState;
      processModeChange();
      return;
    }
  }
  Serial.println ( "not recognised." );
}

void processModeChange() {
  bool chooseRecipe = true;
  switch ( opState ) {
    case OFF:
      pumpState = NORECIRC;
      stepper.moveTo( 0 );
      break;
    case IGNITION:
      pumpState = NORECIRC;
      if ( !ignitionHasOccured ) {
        if ( userResponseWaterFilled || ( !userResponseWaterFilled && getResponseYN( "WATER FILLED?" ) ) ) {
          userResponseWaterFilled = true;
          // Force display update since we are using delay here
          opStateChanged = true;
          actualChanged = true;
          updateDisplay();
          stepper.runToNewPosition( settings.maxPosition );
          delay( 2500 );
          stepper.moveTo( settings.minPosition );
          opState = MAN;
          ignitionHasOccured = true;
        } else {
          opState = OFF;
          processModeChange();
        }
      }
      break;
    case AUTO:
      if ( settings.currentRecipe != -1 ) {
        if ( getResponseYN( "CONTINUE PREV?" ) ) {
          chooseRecipe = false;
        }
      }
      if (chooseRecipe) {
        char choices[NUMBER_OF_RECIPES][16];
        for (int i = 0; i < NUMBER_OF_RECIPES; i++) {
          for (int j = 0; j < 16; j++) {
            choices[i][j] = recipes[i].title[j];
          }
        }
        settings.currentRecipe = getResponseChoices( "CHOOSE RECIPE", choices, 3 );
        settings.currentStep = 0;
        settings.currentHopsAddition = 0;
      }
      stepTargetReached = 0;
      pumpState = ( settings.currentStep < 5 ? RECIRC : NORECIRC );
      if ( String( recipes[settings.currentRecipe].steps[settings.currentStep].startMessage ) != "" ) {
        displayAlert( recipes[settings.currentRecipe].steps[settings.currentStep].startMessage );
      }
      updateSettings();
      break;
    case PUMP:
      pumpState = ( pumpState ? NORECIRC : RECIRC );
      break;
    default:
      break;
  }
  selectedState = opState;
  opStateChanged = true;
  actualChanged = true;
  pumpStateChanged = true;
}

void initBuzzer() {
  pinMode(BUZZER_WIRE_PWR, OUTPUT);
  tone(BUZZER_WIRE_PWR, BUZZER_FREQUENCY);
  delay(20);
  noTone(BUZZER_WIRE_PWR);
}

void initPump() {
  pinMode(PUMP_WIRE_PWR, OUTPUT);
  pinMode(PUMP_WIRE_GND, OUTPUT);
}

void updatePump() {
  digitalWrite(PUMP_WIRE_PWR, (userResponseWaterFilled && pumpState && actual < 84 ? HIGH : LOW ) );
}
