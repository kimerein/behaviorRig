
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>
#include <SD.h>

const long day = 86400000; // 86400000 milliseconds in a day
const long hour = 3600000; // 3600000 milliseconds in an hour
const long minute = 60000; // 60000 milliseconds in a minute
const long second = 1000; // 1000 milliseconds in a second
const uint8_t I2Caddress = 0x60; // use 0x60 as default address
const int chipSelect = 4;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(I2Caddress);

String serialStringOut(String outString, String appendString)
{
  Serial.print(outString);
  appendString += outString;
  return appendString;

}

String time()
{
  long timeNow = millis();
  String outString = "";
  int days = timeNow / day ; //number of days
  int hours = (timeNow % day) / hour; //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
  int minutes = ((timeNow % day) % hour) / minute ; //and so on...
  int seconds = (((timeNow % day) % hour) % minute) / second;

  // digital clock display of current time
  outString = serialStringOut(String(days, DEC), outString);
  outString = serialStringOut(printDigits(hours), outString);
  outString = serialStringOut(printDigits(minutes), outString);
  outString = serialStringOut(printDigits(seconds), outString);
  outString = serialStringOut("\r\n", outString);
  return outString;
}

String printDigits(byte digits)
{
  String outString = "";

  // utility function for digital clock display: prints colon and leading 0
  outString = serialStringOut(":", outString);
  if (digits < 10)
  {
    outString = serialStringOut("0", outString);
  }
  outString = serialStringOut(String(digits, DEC), outString);
  return outString;
}

class DataCard
{

    String dataFile;

  public:
    DataCard(int chpselect, String fileName)
    {
      Serial.print("Initializing SD card ...");
      // see if the card is present and can be initialized
      if (!SD.begin(chpselect))
      {
        Serial.println("Card failed or not present");
        return;
      }
      Serial.println("Card initialized");
      dataFile = fileName;
    }

    void Update(String dataString)
    {
      // open the file
      File data = SD.open(dataFile, FILE_WRITE);
      // if file is available, write to it
      if (data)
      {
        data.println(dataString);
        data.close();
        // print to serial port as well
        //      Serial.println(dataString);
      } else
      {
        Serial.println("Error open SD file");
        Serial.print(">");
        Serial.print(millis());
        Serial.print(">");
        time();
      }
    }
};

DataCard outSD(chipSelect, "output.txt");

class StepperDisk
{
    Adafruit_StepperMotor *myStepper;
    long stepSize; // distance for stepper motor to move per trial
    long updateDelay; // time between steps
    int stepsPerRevolution; // number of steps per revolution
    long motorRPM; // motor speed
    unsigned long lastUpdate; // last update of position
    int stepsSoFar = 0; // number of steps moved on this trial
    int writeCode;
    long delayBeforeTurn = 0; // in ms, delay before movement onset, triggered on trial start
    long trialOnset = 0;
    boolean accomplishedTurn = false;
    boolean goesForward = true;

  public:

    StepperDisk(int nSteps, int whichMotor, int eventLabel, boolean forwardDirection)
    {
      // Connect a stepper motor with nSteps per revolution in position whichMotor
      myStepper = AFMS.getStepper(nSteps, whichMotor);
      stepsPerRevolution = nSteps;
      writeCode = eventLabel;
      goesForward = forwardDirection;
    }

    void SetAngle(int nSteps)
    {
      if (goesForward == true)
      {
        myStepper->step(nSteps, FORWARD, MICROSTEP);
      } else
      {
        myStepper->step(nSteps, BACKWARD, MICROSTEP);
      }
    }

    void SetSpeed(long rpm)
    {
      myStepper->setSpeed(rpm); // in rotations per minute
      motorRPM = rpm;
    }


    void Update(long currStepSize, long waitToTurn, boolean trialStarting)
    {
      if (trialStarting == true) // start of a new trial
      {
        // set goal distance for motor to move on this trial
        stepSize = currStepSize;
        // at given rpm speed, calculate update delay
        updateDelay = (1 / stepsPerRevolution) * (60 / motorRPM); // in seconds
        stepsSoFar = 0; // reset to zero
        // set delay before motor starts turning
        delayBeforeTurn = waitToTurn;
        trialOnset = millis();
        accomplishedTurn = false;
      }
      if ((millis() - trialOnset) > delayBeforeTurn) // time to start turning
      {
        if ((millis() - lastUpdate) > updateDelay) // time to update
        {
          lastUpdate = millis();
          if (stepsSoFar < stepSize)
          {
            if (stepsSoFar == 0) // starting turn
            {
              String outString = "";
              outString = serialStringOut(String(writeCode), outString);
              outString = serialStringOut(">S>", outString);
              outString = serialStringOut(String(millis()), outString);
              outString = serialStringOut("\r\n", outString);
              outSD.Update(outString);
            }
            if (goesForward == true)
            {
              myStepper->step(1, FORWARD, MICROSTEP);
            } else
            {
              myStepper->step(1, BACKWARD, MICROSTEP);
            }
            stepsSoFar += 1; // moved one more step
          } else if ((stepsSoFar >= stepSize) && (accomplishedTurn == false)) // ending turn
          {
            if (currStepSize == 1)
            {
              if (goesForward == true)
              {
                delay(500);
                myStepper->step(1, FORWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, FORWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, FORWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, FORWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, FORWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, FORWARD, INTERLEAVE);
              } else
              {
                delay(500);
                myStepper->step(1, BACKWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, BACKWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, BACKWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, BACKWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, BACKWARD, INTERLEAVE);
                delay(500);
                myStepper->step(1, BACKWARD, INTERLEAVE);
              }
            } 
            accomplishedTurn = true;
          }
        }
      }
    }
};

class FlashOnce
{

    int ledPin; // analog output pin number
    long OnTime; // milliseconds of on-time
    long OffTime; // milliseconds of off-time
    int ledState; // led state
    unsigned long previousMillis; // stores last time LED was updated
    boolean didOnce = false; // whether already flashed once
    int writeCode;

  public:
    FlashOnce(int pin, long on, long off, int eventLabel)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);
      OnTime = on;
      OffTime = off;
      ledState = HIGH; // start off
      digitalWrite(ledPin, ledState); // Update the actual LED
      previousMillis = 0;
      writeCode = eventLabel;
    }

    void Update(boolean trialStarting)
    {
      unsigned long currentMillis = millis();
      if (trialStarting == true)
      {
        // re-initialize so can turn on again
        didOnce = false;
        // check that LED is off
        ledState = HIGH;
        digitalWrite(ledPin, ledState); // Update the actual LED
        previousMillis = currentMillis;
      }
      if ((ledState == LOW) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = HIGH; // Turn it off
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">E>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        //      outString = serialStringOut(">", outString);
        //      outString = serialStringOut(time(), outString);
        outSD.Update(outString);
      }
      else if ((ledState == HIGH) && (currentMillis - previousMillis >= OffTime) && (didOnce == false)) // Haven't yet turned LED on
      {
        ledState = LOW; // Turn it on
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">S>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        //      outString = serialStringOut(">", outString);
        //      outString = serialStringOut(time(), outString);
        outSD.Update(outString);
        didOnce = true; // Save that turned on once
      }
    }
};

class FlashRandom
{

    int ledPin; // analog output pin number
    long OnTime; // milliseconds of on-time
    long OffTime; // milliseconds of off-time
    int ledState; // led state
    int minDelay; // minimum time delay between random LED transitions, in milliseconds
    int maxDelay; // maximum time delay between random LED transitions, in milliseconds
    unsigned long previousMillis; // stores last time LED was updated
    int writeCode;
    long delayToCue; // delay from start of trial to cue in milliseconds
    boolean haveReset = false; // whether have reset distractor at end of wheel turning
    long distractP; // probability that distractor LED is on versus off
    long timeAtTurn;
    long cueOn;
    long newrand;

  public:
    FlashRandom(int pin, int unconnectedPin, int minRand, int maxRand, int eventLabel, long off, long distractorP, long cueDuration)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);
      minDelay = minRand;
      maxDelay = maxRand;
      // OnTime = random(minRand, maxRand);
      OnTime = cueDuration;
      OffTime = random(minRand, maxRand);
      ledState = HIGH;
      digitalWrite(ledPin, ledState); // Update the LED
      previousMillis = 0;
      writeCode = eventLabel;
      delayToCue = off;
      distractP = distractorP;
      cueOn = cueDuration;
    }

    void Update(boolean trialStarting)
    {
      unsigned long currentMillis = millis();
      if (trialStarting == true)
      {
        // turn off LED
        timeAtTurn = currentMillis;
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">E>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
        haveReset = false;
      }
      else if (currentMillis - timeAtTurn < delayToCue) // pellet wheel is turning
      {
        // leave LED off
      }
      else if ((currentMillis - timeAtTurn >= delayToCue) && (haveReset == false))
      {
        // use distractP to determine probability that LED turns back on
        newrand = random(0, 1);
        if (newrand < distractP)
        {
          // turn LED on
          ledState = LOW;
          digitalWrite(ledPin, ledState); // Update the actual LED
          String outString = "";
          outString = serialStringOut(String(writeCode), outString);
          outString = serialStringOut(">S>", outString);
          outString = serialStringOut(String(millis()), outString);
          outString = serialStringOut("\r\n", outString);
          outSD.Update(outString);
          previousMillis = currentMillis; // Remember the time
          OnTime = cueOn; // in ms
          OffTime = random(minDelay, maxDelay); // in ms
        }
        haveReset = true;
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = HIGH; // Turn it off
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">E>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
        OnTime = cueOn; // in ms
        OffTime = random(minDelay, maxDelay); // in ms
      }
      else if ((ledState == HIGH) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = LOW; // Turn it on
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">S>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
      }
    }
};

class ThresholdSensor
{

    long threshold = 1;
    int sensorPin;
    int count = 0;
    int writeCode;

  public:
    ThresholdSensor(int pin, long thresh, int eventLabel)
    {
      threshold = thresh;
      sensorPin = pin;
      writeCode = eventLabel;
    }

    void Update()
    {
      long sensorReading = analogRead(sensorPin);
      // if sensor reading is greater than threshold
      if (sensorReading >= threshold)
      {
        count += 1; // increment count
      }
    }

    int GetCount()
    {
      return count;
    }
};

// Declare constants
const int loaderN = 1;
const int pelletsN = 2;
const uint8_t motorShieldAddress = 0x60;
const int stepsForStepper = 200;
const int cuePin = 6; // digital output pin that controls cue
const long cueDuration = 250; // cue on duration in ms
const long cueDelay = 1500; // delay from trial start until cue turns on in ms
const int distractorPin = 2; // digital output pin that controls distractor
const int emptyPin = 4; // an analog input pin that is not connected (for random seed initialization)
const int randomMin = 1000; // in ms
const int randomMax = 10000; // in ms
const long probabilityDistractor = 0.05; // probability that distractor will be on versus off
const int loaderWriteCode = 1;
const int pelletsWriteCode = 2;
const int cueWriteCode = 4;
const int distractorWriteCode = 5;
const int beginTrialWrite = 0;
const long minITI = 25000;
const long maxITI = 35000;
const long pelletsDelay = 0; // in ms
const long loaderDelay = 5000; // in ms

long stepperMotorRPM = 100; // rpm
long loaderMotorRPM = 20; // rpm
boolean trialIsStarting = false;
int trialState = LOW;
long ITI = 0;
unsigned long prevMs = 0;
unsigned long currMs = 0;
int nTrials = 0;

// probabilityDistractor for FlashRandom should match ratio of cueDuration to expected value of [randomMin, randomMax]
FlashRandom distractor(distractorPin, emptyPin, randomMin, randomMax, distractorWriteCode, cueDelay + cueDuration, probabilityDistractor, cueDuration);
FlashOnce cue(cuePin, cueDuration, cueDelay, cueWriteCode);
StepperDisk loader(stepsForStepper, loaderN, loaderWriteCode, true);
StepperDisk pellets(stepsForStepper, pelletsN, pelletsWriteCode, true);

void setup()
{
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.println("Program Begins");
  AFMS.begin();
  loader.SetSpeed(loaderMotorRPM);
  pellets.SetSpeed(stepperMotorRPM);
}

void loop()
{
  currMs = millis();
  // if ITI has elapsed, initiate new trial
  if ((trialState == LOW) && ((currMs - prevMs) > ITI))
  {
    prevMs = currMs;
    trialState = HIGH;
    ITI = random(minITI, maxITI);
    // write ITI
    String outputString = "";
    outputString = serialStringOut(String(beginTrialWrite), outputString);
    outputString = serialStringOut(">", outputString);
    outputString = serialStringOut(String(ITI), outputString);
    outputString = serialStringOut(">", outputString);
    outputString = serialStringOut(String(millis()), outputString);
    outputString = serialStringOut("\r\n", outputString);
    outSD.Update(outputString);
    // increment trial count
    nTrials += 1;
  } else if (trialState == HIGH)
  {
    trialState = LOW;
  }

  if (trialState == LOW)
  {
    trialIsStarting = false;
  } else if (trialState == HIGH)
  {
    trialIsStarting = true;
  }
  loader.Update(1, loaderDelay, trialIsStarting);
  pellets.Update(20, pelletsDelay, trialIsStarting);
  cue.Update(trialIsStarting);
  distractor.Update(trialIsStarting);
}


