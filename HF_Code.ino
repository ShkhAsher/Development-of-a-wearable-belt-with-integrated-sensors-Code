/******************************************************************************
  HF paper code that aquire data from impedance (AD5933) sensor,Heart Rate (MAX303105) sensor, ECG (AD8232) sensor and Accelerometer (ADXL362) sensor using
  AD5933.h, MAX30105.h, heartRate, and ADXL362.h libraries.
  Arduino MKR 1010 has been used as a microcontoller and its TX1 and RX1 pins have been used for sending data over the bluetooth Serial Terminal using HC-05 bluetooth module.
  For more information about the sensors use Readme file.

  /******************************************************************************
  Heart_Rate_Display.ino
  Demo Program for AD8232 Heart Rate sensor.
  Casey Kuhns @ SparkFun Electronics
  6/27/2014
  https://github.com/sparkfun/AD8232_Heart_Rate_Monitor
  The AD8232 Heart Rate sensor is a low cost EKG/ECG sensor.  This example shows
  how to create an ECG with real time display.  The display is using Processing.
  This sketch is based heavily on the Graphing Tutorial provided in the Arduino
  IDE. http://www.arduino.cc/en/Tutorial/Graph
  Resources:
  This program requires a Processing sketch to view the data in real time.
  Development environment specifics:
  IDE: Arduino 1.0.5

******************************************************************************/
#include <WString.h>
#include <SPI.h>
#include <SD.h>
//#include <LowPower.h>
//#include <ArduinoBLE.h>
#include <TimeLib.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "AD5933.h" //AD5933 Header file
#include <ADXL362.h>




int Hour = 0;
int Minute = 0;
int Second = 0;


File myFile;
File myFile1;
File myFile2;
File myFile3;


//ADXL362
ADXL362 xl;

int16_t temp;
//  Setup interrupt on Arduino
int16_t interruptPin = 2;          //Setup ADXL362 interrupt output to Interrupt 0 (digital pin 2)
int16_t interruptStatus = 0;
int test = 0;
int16_t XValue, YValue, ZValue, Temperature; //// MAX 30105

////////////////////////////////////
#define START_FREQ  (80000)
#define FREQ_INCR   (1000)
#define NUM_INCR    (20)
#define REF_RESIST  (82)

double gain[NUM_INCR + 1];
int phase[NUM_INCR + 1];
/////////////////////////////////

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
///

void setup() {


  // ADXL362

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("Start  Demo: Simple Read");
  Serial1.println("Start  Demo: Simple Read");



  xl.begin(6);                   // Setup SPI protocol, issue device soft reset
  xl.beginMeasure();              // Switch ADXL362 to measure mode
  xl.setupDCActivityInterrupt(300, 10);   // 300 code activity threshold.  With default ODR = 100Hz, time threshold of 10 results in 0.1 second time threshold
  xl.setupDCInactivityInterrupt(80, 200);   // 80 code inactivity threshold.  With default ODR = 100Hz, time threshold of 30 results in 2 second time threshold
  Serial.println();
  Serial1.println();



  // Setup ADXL362 for proper autosleep mode
  //

  // Map Awake status to Interrupt 1
  // *** create a function to map interrupts... coming soon
  xl.SPIwriteOneRegister(0x2A, 0x40);

  // Setup Activity/Inactivity register
  xl.SPIwriteOneRegister(0x27, 0x3F); // Referenced Activity, Referenced Inactivity, Loop Mode

  // turn on Autosleep bit
  byte POWER_CTL_reg = xl.SPIreadOneRegister(0x2D);
  POWER_CTL_reg = POWER_CTL_reg | (0x04);       // turn on POWER_CTL[2] - Autosleep bit
  xl.SPIwriteOneRegister(0x2D, POWER_CTL_reg);




  //
  // turn on Measure mode
  //
  xl.beginMeasure();                      // DO LAST! enable measurement mode
  xl.checkAllControlRegs();               // check some setup conditions
  delay(100);



  //
  // Setup interrupt function on Arduino
  //    IMPORTANT - Do this last in the setup, after you have fully configured ADXL.
  //    You don't want the Arduino to go to sleep before you're done with setup
  //
  pinMode(2, INPUT);
  attachInterrupt(0, interruptFunction, RISING);  // A high on output of ADXL interrupt means ADXL is awake, and wake up Arduino

  //////////ECG////////
  pinMode(4, INPUT); // Setup for leads off detection LO + change it to 8
  pinMode(5, INPUT); // Setup for leads off detection LO - change it to 9
  //////Heart Rate//////////
  Serial.println("Initializing...");
  Serial1.println("Initializing...");


  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    Serial1.println("MAX30105 was not found. Please check wiring/power. ");

    while (1);
  }
  else {
    Serial.println("Success");
    Serial1.println("Success");


  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
  Serial1.println("Place your index finger on the sensor with steady pressure.");


  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  ///////////AD5933////////////////
  Serial.println("AD5933 Test Started!");
  Serial1.println("AD5933 Test Started!");


  // Perform initial configuration. Fail if any one of these fail.
  if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
  {
    Serial.println("FAILED in initialization!");
    Serial1.println("FAILED in initialization!");

    while (true) ;
  }

  // Perform calibration sweep
  if (AD5933::calibrate(gain, phase, REF_RESIST, NUM_INCR + 1))
  { Serial.println("Calibrated!");
    Serial1.println("Calibrated!");
  }
  else
  { Serial.println("Calibration failed...");
    Serial1.println("Calibration failed...");
  }

}
void loop() {

  delay(500);   // Delay

  //SD card

  if (!SD.begin(7)) {
    Serial.println("initialization failed!");
    Serial1.println("initialization failed!");

  }
  else
  { Serial.println("initialization done.");
    Serial1.println("initialization done.");
  }

  myFile = SD.open("test.txt", FILE_WRITE);

  if (myFile) {
    Serial.print("Writing to test.txt...");
    Serial1.print("Writing to test.txt...");




    //Activity Motion//
    digitalClockDisplay();


    interruptStatus = digitalRead(interruptPin);

    // if ADXL362 is asleep, call LowPower.powerdown
    if (interruptStatus == 0) {
      Serial.print("\nADXL went to sleep - Put Arduino to sleep now \n");
      Serial1.print("\nADXL went to sleep - Put Arduino to sleep now \n");

      myFile.println("\nADXL went to sleep - Put Arduino to sleep now \n");

      digitalWrite(7, LOW);    // Turn off LED as visual indicator of asleep
      delay(100);
    }

    // if ADXL362 is awake, report XYZT data to Serial Monitor
    else {
      delay(10);
      digitalWrite(7, HIGH);    // Turn on LED as visual indicator of awake
      xl.readXYZTData(XValue, YValue, ZValue, Temperature);
      myFile.println("\nADXL is awake");
      Serial.println("awake");
      Serial1.println("awake");

    }
    // give circuit time to settle after wakeup
    delay(20);
    myFile.close();
  }
  else {
    Serial.println("error");
    Serial1.println("error");
  }
  //Thoracic Impedance//
  myFile1 = SD.open("test1.txt", FILE_WRITE);
  if (myFile1) {
    Serial.print("Writing to test1.txt...");
    Serial1.print("Writing to test1.txt...");


    frequencySweepRaw();
    myFile1.close();
    Serial.println("done1.");
    Serial1.println("done1.");

    delay(1000);
  }
  else {
    Serial.println("error1");
    Serial1.println("error1");
  }
  //ECG//
  myFile2 = SD.open("test2.txt", FILE_WRITE);
  if (myFile2) {
    Serial.print("Writing to test2.txt...");
    Serial1.print("Writing to test2.txt...");

    ecg();
    myFile2.close();
    Serial.println("done2.");
    Serial1.println("done2.");

  }
  else {
    Serial.println("error2");
    Serial1.println("error2");
  }
  myFile3 = SD.open("test3.txt", FILE_WRITE);
  //Heart Rate//
  if (myFile3) {
    Serial.print("Writing to test3.txt...");
    Serial1.print("Writing to test3.txt...");

    hr();

    myFile3.close();
    Serial.println("done3.");
    Serial1.println("done3.");
  }

  else {
    Serial.println("error3");
    Serial1.println("error3");
  }

  // Delay
  delay(500);

}

//Acuqiring Data from AD5933//

void frequencySweepRaw() {
  // Create variables to hold the impedance data and track frequency
  double imp = 0;
  double avg_imp = 0;

  int real, imag, i = 0, cfreq = START_FREQ / 1000;

  // Initialize the frequency sweep
  if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
        AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
        AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
  {
    Serial.println("Could not initialize frequency sweep...");
  }

  // Perform the actual sweep
  while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
    // Get the frequency data for this frequency point
    if (!AD5933::getComplexData(&real, &imag)) {
      Serial.println("Could not get raw frequency data...");
    }

    // Print out the frequency data


    Serial.print(cfreq);
    Serial.print(": R=");
    Serial.print(real);
    Serial.print("/I=");
    Serial.print(imag);

    Serial1.print(cfreq);
    Serial1.print(": R=");
    Serial1.print(real);
    Serial1.print("/I=");
    Serial1.print(imag);


    digitalClockDisplay1();
    myFile1.print(cfreq);
    myFile1.print(": R=");
    myFile1.print(real);
    myFile1.print("/I=");
    myFile1.print(imag);


    // Compute impedance
    double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
    double impedance = 1 / (magnitude * gain[i]);
    imp = imp + impedance;
    Serial.print("  |Z|=");
    Serial.println(impedance);
    Serial1.print("  |Z|=");
    Serial1.println(impedance);

    digitalClockDisplay1();

    myFile1.print("  |Z|=");
    myFile1.println(impedance);

    // Increment the frequency
    i++;
    cfreq += FREQ_INCR / 1000;
    AD5933::setControlMode(CTRL_INCREMENT_FREQ);
  }
  digitalClockDisplay1();
  avg_imp = imp / i;
  Serial.println(i);
  Serial.println("Avg Impedance");
  Serial.print(avg_imp);
  myFile1.print("  Avg |Z|=");
  myFile1.println(avg_imp);
  Serial1.println("Avg Impedance");
  Serial1.print(avg_imp);

  Serial.println("Frequency sweep complete!");
  Serial1.println("Frequency sweep complete!");


  // Set AD5933 power mode to standby when finished
  if (!AD5933::setPowerMode(POWER_STANDBY))
  {
    Serial.println("Could not set to standby...");
    Serial1.println("Could not set to standby...");

  }
}
//Acuqiring Data from AD8232//

void ecg()
{
  for (int i = 0 ; i < 750 ; i++)
  {

    if ((digitalRead(4) == 1) || (digitalRead(5) == 1)) {
      digitalClockDisplay2();
      Serial.println('!');
      Serial1.println('!');

      myFile2.println('!');
    }
    else {
      digitalClockDisplay2();
      // send the value of analog input 0:
      Serial.println(analogRead(A0));
      Serial1.println(analogRead(A0));

      myFile2.println(analogRead(A0));
    }
    //Wait for a bit to keep serial data from saturating
    delay(1);

    ////
  }
  delay(53);
}

//Acuqiring Data from MAX30105//

void hr() {


  for (int i = 0 ; i < 250 ; i++)
  {
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }


    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial1.print("IR=");
    Serial1.print(irValue);
    Serial1.print(", BPM=");
    Serial1.print(beatsPerMinute);
    Serial1.print(", Avg BPM=");
    Serial1.print(beatAvg);
    digitalClockDisplay3();
    myFile3.print(F("IR="));
    myFile3.print(irValue);
    myFile3.print(", BPM=");
    myFile3.print(beatsPerMinute);
    myFile3.print(", Avg BPM=");
    myFile3.print(beatAvg);

    if (irValue < 50000)
    { Serial.print(" No finger?");

      myFile3.print(" No finger?");
    }

    Serial.println();
    Serial1.println();

    myFile3.println();


  }
}

//Time Stamp functions

void digitalClockDisplay() {
  // digital clock display of the time
  myFile.print(hour());
  printDigits(minute());
  printDigits(second());

  myFile.print(" ");
}
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  myFile.print(":");
  if (digits < 10)
    myFile.print('0');
  myFile.print(digits);
}
void digitalClockDisplay1() {
  // digital clock display of the time
  myFile1.print(hour());
  printDigits1(minute());
  printDigits1(second());

  myFile1.print(" ");
}
void printDigits1(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  myFile1.print(":");
  if (digits < 10)
    myFile1.print('0');
  myFile1.print(digits);
}
void digitalClockDisplay2() {
  // digital clock display of the time
  myFile2.print(hour());
  printDigits2(minute());
  printDigits2(second());

  myFile2.print(" ");
}
void printDigits2(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  myFile2.print(":");
  if (digits < 10)
    myFile2.print('0');
  myFile2.print(digits);
}

void digitalClockDisplay3() {
  // digital clock display of the time
  myFile3.print(hour());
  printDigits3(minute());
  printDigits3(second());

  myFile3.print(" ");
}
void printDigits3(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  myFile3.print(":");
  if (digits < 10)
    myFile3.print('0');
  myFile3.print(digits);
}
void interruptFunction() {
  Serial.println("\nArduino is Awake! \n");
}
