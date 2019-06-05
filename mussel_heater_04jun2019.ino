
//This runs TANK xx with 9 mussels
//last updated 5.28.19

#include <math.h>

//WD 04jun2019
//bool serialEcho = 1; //1 to write information to Serial for troubleshooting
//                     //0 to supress
                     
//placeholder strings
String inputData = "";  //placeholder for 51-char string coming from Matlab //W34.0035.0036.0037.0038.0039.0040.0041.0042.007.00

#define numMussels 9

//placeholder temps; if change numMussels must change next 2 lines of code to initialize array properly
double desiredTemps[numMussels] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //, 0, 0, 0, 0, 0, 0  <--add within brackets if going to 12 mussels
double recordedTemps[numMussels] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; //, 0, 0, 0, 0, 0, 0 <--add within brackets if going to 12 mussels

//placeholders for pins on Arduino, assigned in setup loop
int thermPins[numMussels]; //empty array for thermistor pins, filled in later
int heatPins[numMussels];  //empty array for designating heater pins, filled in later
double thermResistor[numMussels]; //empty array for thermistor resistance values
int echoPinA; //pin for reading height tank A
int trigPinA; //output to height sensor tankA
int valveA; //output pin to tank A water valve

//default values for water height setpoint/reading
double desiredLevelA = -2; // Desired Water Level for Tank A
double recordedLevelA = 0; // Recorded Water Level for Tank A

//WD 04jun2019 moved these parameters to setup rather than resetting each time in main loop
int dataLength = 0;
int checkDataLength = numMussels * 5 +4+1+1; //WD 04jun2019: 1 letter for tank, numMussels x 5 for settemp, 4 setlevel, 1nl
char tank ='W'; //placeholder
double desiredM1 =0;
double desiredM2 =0;
double desiredM3 =0;
double desiredM4 =0;
double desiredM5 =0;
double desiredM6 =0;
double desiredM7 =0;
double desiredM8 =0;
double desiredM9 =0;
double desiredWL =0;

// SETUP PARAMETERS AND COMMUNICATION ------------------------------------------
void setup() {
  thermPins[0] = A0; // Thermistor Pin for Mussel 0
  thermPins[1] = A1; // Thermistor Pin for Mussel 1
  thermPins[2] = A2; // Thermistor Pin for Mussel 2
  thermPins[3] = A3; // Thermistor Pin for Mussel 3
  thermPins[4] = A4; // Thermistor Pin for Mussel 4
  thermPins[5] = A5; // Thermistor Pin for Mussel 5
  thermPins[6] = A6; // Thermistor Pin for Mussel 6
  thermPins[7] = A7; // Thermistor Pin for Mussel 7
  thermPins[8] = A8; // Thermistor Pin for Mussel 8
 

  heatPins[0] = 0; // Heat Pad Pin for Mussel 0
  heatPins[1] = 1; // Heat Pad Pin for Mussel 1
  heatPins[2] = 2; // Heat Pad Pin for Mussel 2
  heatPins[3] = 3; // Heat Pad Pin for Mussel 3
  heatPins[4] = 4; // Heat Pad Pin for Mussel 4
  heatPins[5] = 5; // Heat Pad Pin for Mussel 5
  heatPins[6] = 6; // Heat Pad Pin for Mussel 6
  heatPins[7] = 7; // Heat Pad Pin for Mussel 7
  heatPins[8] = 8; // Heat Pad Pin for Mussel 8

//WD NOTE: empirically measure resistance with multimeter in lab and update here
  thermResistor[0] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 0
  thermResistor[1] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 1
  thermResistor[2] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 2
  thermResistor[3] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 3
  thermResistor[4] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 4
  thermResistor[5] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 5
  thermResistor[6] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 6
  thermResistor[7] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 7
  thermResistor[8] = 10200.0; // Value of the Resistor in series with the thermistor for Mussel 8

  echoPinA = 10; // Echo Pin for Tank A height sensor
  trigPinA = 11; // Trig Pin for Tank A height sensor

  valveA = 9; // Pin for the Electronic Valve for Tank A



  //set thermPins and heatPins as input/output, respectively
  for (int i = 0; i < numMussels; i++) {
    pinMode(thermPins[i], INPUT);
    pinMode(heatPins[i], OUTPUT);
  }


  // Sets up heater and valve pin data directions
  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);

  pinMode(valveA, OUTPUT);

  // Sets up serial communication with MATLAB and checks
  // it with a serial communication 'handshake'
  Serial.begin(115200); //changed from 9600 because of a bug 5.29.19
  delay(2000); // wait for console opening WD 04jun2019
  Serial.read();  //clear serial buffer
 
  char a = 'b';
  while (a != 'a') {  //waits for string 'a' to be sent from Matlab, how fancy!
  a = Serial.read();
  }
  //WD 04jun2019 moved this so Arduino waits for 'a' from Matlab/serial before sending an 'a' back
  Serial.println('a');  //possibly Serial.write //Serial.write('a');
  Serial.read(); //clear serial buffer WD 04jun2019
}

// MAIN EXECUTION LOOP------------------------------------------
void loop() {

 

  //FILL VARIABLE FIELDS WITH SENSOR READINGS A PRIORI, THIS WILL DEAL WITH INITIAL 0's in MATLAB DISPLAY TABLE (I THINK...)
  update();  //this might screw things up??
 
  //build command string 'inputData' one character at a time
  while (Serial.available() > 0) {
    while (dataLength < checkDataLength) {
    //format is five characters for each temperature in celsius (XX.XX) x numMussels + X.XX for water level
      char received = Serial.read();
      inputData += received;
      dataLength = inputData.length();
      //Serial.println(dataLength);  //WD just checking to see that right length accumulated from serial, it is
    }
  }
  //
  if (dataLength == checkDataLength) {    //   WD 04jun2019 replaced: if (dataLength > 1) {
    //Serial.println(inputData); //uncomment to check the received command string format in Arduino serial monitor
    //break up the string received into tank, desired mussel temps (9), and water level
    //tank = inputData.charAt(0);
    tank = inputData[0];  //WD 04jun2019 this comes in as a character byte anyway in first position (0)
    // WD 04jun2019 check inputs
//    Serial.print(F("Tank: "));
//    Serial.println(tank);
   
//    desiredM1 = inputData[1,5];
//    desiredM2 = inputData[6,10];
//    desiredM3 = inputData[11,15];
//    desiredM4 = inputData[16,20];
//    desiredM5 = inputData[21,25];
//    desiredM6 = inputData[26,30];
//    desiredM7 = inputData[31,35];
//    desiredM8 = inputData[36,40];
//    desiredM9 = inputData[41,45];
//    desiredWL = inputData[46,dataLength];

// WD 04jun2019 these come in as digits in inputData, and must be Double to match how desiredM1, etc were declared    
    desiredM1 = (inputData.substring(1, 5)).toFloat();
    desiredM2 = (inputData.substring(6, 10)).toFloat();
    desiredM3 = (inputData.substring(11, 15)).toFloat();
    desiredM4 = (inputData.substring(16, 20)).toFloat();
    desiredM5 = (inputData.substring(21, 25)).toFloat();
    desiredM6 = (inputData.substring(26, 30)).toFloat();
    desiredM7 = (inputData.substring(31, 35)).toFloat();
    desiredM8 = (inputData.substring(36, 40)).toFloat();
    desiredM9 = (inputData.substring(41, 45)).toFloat();
    desiredWL = (inputData.substring(46, dataLength)).toFloat();
   
    //WD 04jun2019 for testing uncomment next 3 lines
//    printDouble(desiredM1, 2);
//    printDouble(desiredM2, 2);
//    printDouble(desiredWL, 2);
//    Serial.println("");
    //Serial.println(desiredM2)); //for testing

    //implement heat control and water level control
    desiredTemps[0] = desiredM1;
    desiredTemps[1] = desiredM2;
    desiredTemps[2] = desiredM3;
    desiredTemps[3] = desiredM4;
    desiredTemps[4] = desiredM5;
    desiredTemps[5] = desiredM6;
    desiredTemps[6] = desiredM7;
    desiredTemps[7] = desiredM8;
    desiredTemps[8] = desiredM9;
    desiredLevelA = desiredWL;
    heatControl(desiredTemps[0], recordedTemps[0], heatPins[0]);
    heatControl(desiredTemps[1], recordedTemps[1], heatPins[1]);
    heatControl(desiredTemps[2], recordedTemps[2], heatPins[2]);
    heatControl(desiredTemps[3], recordedTemps[3], heatPins[3]);
    heatControl(desiredTemps[4], recordedTemps[4], heatPins[4]);
    heatControl(desiredTemps[5], recordedTemps[5], heatPins[5]);
    heatControl(desiredTemps[6], recordedTemps[6], heatPins[6]);
    heatControl(desiredTemps[7], recordedTemps[7], heatPins[7]);
    heatControl(desiredTemps[8], recordedTemps[8], heatPins[8]);
    waterControl(desiredLevelA, recordedLevelA, valveA);
   
 
    //return actual temps and water level to MATLAB
    //get actual temp using the smoothing function
   
    Serial.print(tank);
    Serial.print("\t");
    recordedTemps[0] = smoothing(60, thermPins[0], thermResistor[0]);
    recordedTemps[1] = smoothing(60, thermPins[1], thermResistor[1]);
    recordedTemps[2] = smoothing(60, thermPins[2], thermResistor[2]);
    recordedTemps[3] = smoothing(60, thermPins[3], thermResistor[3]);
    recordedTemps[4] = smoothing(60, thermPins[4], thermResistor[4]);
    recordedTemps[5] = smoothing(60, thermPins[5], thermResistor[5]);
    recordedTemps[6] = smoothing(60, thermPins[6], thermResistor[6]);
    recordedTemps[7] = smoothing(60, thermPins[7], thermResistor[7]);
    recordedTemps[8] = smoothing(60, thermPins[8], thermResistor[8]);
    printDouble(recordedTemps[0], 2);
    printDouble(recordedTemps[1], 2);
    printDouble(recordedTemps[2], 2);
    printDouble(recordedTemps[3], 2);
    printDouble(recordedTemps[4], 2);
    printDouble(recordedTemps[5], 2);
    printDouble(recordedTemps[6], 2);
    printDouble(recordedTemps[7], 2);
    printDouble(recordedTemps[8], 2);
    printDouble(recordedLevelA, 2);
    Serial.println("");
   
    inputData = ""; // Clear received Matlab command string before looking for serial again
    dataLength = 0;
    //break; //this is new 5.30.19
  }
  //this used to be in an "else" statement, accompanying the previous "if" loop that is now "while"
  update();
  //Serial.println(desiredTemps[3]); //for testing
}



// FUNCTIONS ------------------------------------------
// ------------------------------------------

// Function that gets the temperature of a thermistor given its pin.
// use this in smoothing function
double getTemperature(int thermPin, double R) {

  // TEMPERATURE PROBE SETUP DATA
  // Don't change the equations, just these values
  double R0 = 10000.0; // R0 is the thermistor resistance at T0 (25 Celsius)
  double T0 = 25.0; // T0 in Celsius
  double B = 3800.0; // B Constant of Thermistor

  // Reads in a value from 0-1023. Converts that to the voltage.
  double thermVolt = analogRead(thermPin) * (5.0 / 1023.0);

  // Uses voltage divider equation to determine the resistance.
  double thermResis = (R * thermVolt) / (5 - thermVolt);

  // Uses the Steinhart–Hart equation to determine the temp in K.
  double tempKelvin = B / (log(thermResis / (R0 * exp(-B / (T0 + 273.15)))));

  // Converts the temp in K to C.
  double tempCel = tempKelvin - 273.15;
  return tempCel;
}

// ------------------------------------------

// Function that gets the distance from the ultrasonic sensor given its pins.
double getDistance(int trigPin, int echoPin) {
  float duration, distance, height, averageTotal, average, points;
  points = 100;
  averageTotal = 0;
  int i = 0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = ((duration / 2) / 29.1) - 1;
  height = 17 - distance;
  averageTotal = averageTotal + height;
  average = averageTotal / points;
  if (height < 0) {
    height = 0;
  }
  if (height > 9) {
    height = 9;
  }
  return height;
}

// ------------------------------------------

void printDouble( double val, byte precision) {
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)
  // NEW: also prints a tab at the end so MATLAB can parse a bunch of numbers into an array

  Serial.print (int(val));  //prints the int part
  if ( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;
    while ( frac1 /= 10 )
      padding--;
    while (  padding--)
      Serial.print("0");
    Serial.print(frac, DEC) ;
  }
  Serial.print("\t");
}

// ------------------------------------------

void heatControl(double desiredTemp, double actualTemp, int heatPin) {
  double targetOut;
  double perDiff = percentDiff(desiredTemp, actualTemp);
  double slope = 255/100; //changed so yint is 0 - shouldn't they always go through 0??
  double output = slope*perDiff;
  if (output > 255) {
    targetOut = 255;
  }
  if (output < 0) {
    targetOut = 0;
  }
  else {
    targetOut = output + 90;
  }
  if (targetOut > 255) {
    targetOut = 255;
  }
  analogWrite(heatPin,targetOut);
  //Serial.println(targetOut);
  //Serial.println(perDiff);
}

// ------------------------------------------
//Smoothing function, returns average temp over a certain number of readings for a given thermistor
//uses getTemperature function
double smoothing(int numReadings, int thermPin, double R) {  
  double getTemp = 0;               // one temp reading, gets written over
  double total = 0;                  // the running total
  double average = 0;                // the average
  //loops over this
  for (int i=0; i <= numReadings; i++) {
      // read from the sensor:
      getTemp = getTemperature(thermPin, R);
      // add the reading to the total:
      total = total + getTemp;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  return average;
  delay(1);        // delay in between reads for stability
}

// ------------------------------------------

// Function that returns the percent difference between two numbers.
// If the result is positive, X > Y
// If the result is negative, X < Y
double percentDiff(double x, double y) {
  double result = (x - y)/(y) * 100;
  return result;
}

// ------------------------------------------

void waterControl(double desiredLevel, double recordedLevel, int valvePin) {
  if (recordedLevel < desiredLevel) {
    // Open Valve
    analogWrite(valvePin, 255);
  }
  else {
    //Close Valve
    analogWrite(valvePin, 0);
  }
}

// ------------------------------------------

void update() {   //update function that runs at the beginning of each loop

  for (int i = 0; i < numMussels; i++) {
    // Go through and take temperature measurements for each mussel
    //smooths between 10 measurements
    recordedTemps[i] = smoothing(60, thermPins[i], thermResistor[i]);
    //Serial.println(recordedTemps[i]);
  }

  // Record water level for the tank
  recordedLevelA = getDistance(trigPinA, echoPinA);

  //update things with last recorded value from MATLAB
  heatControl(desiredTemps[0], recordedTemps[0], heatPins[0]);
  heatControl(desiredTemps[1], recordedTemps[1], heatPins[1]);
  heatControl(desiredTemps[2], recordedTemps[2], heatPins[2]);
  heatControl(desiredTemps[3], recordedTemps[3], heatPins[3]);
  heatControl(desiredTemps[4], recordedTemps[4], heatPins[4]);
  heatControl(desiredTemps[5], recordedTemps[5], heatPins[5]);
  heatControl(desiredTemps[6], recordedTemps[6], heatPins[6]);
  heatControl(desiredTemps[7], recordedTemps[7], heatPins[7]);
  heatControl(desiredTemps[8], recordedTemps[8], heatPins[8]);
  waterControl(desiredLevelA, recordedLevelA, valveA);

  // this is new 5.30.19
//  printDouble(recordedTemps[0], 2);
//  printDouble(recordedTemps[1], 2);
//  printDouble(recordedTemps[2], 2);
//  printDouble(recordedTemps[3], 2);
//  printDouble(recordedTemps[4], 2);
//  printDouble(recordedTemps[5], 2);
//  printDouble(recordedTemps[6], 2);
//  printDouble(recordedTemps[7], 2);
//  printDouble(recordedTemps[8], 2);
//  printDouble(recordedLevelA, 2);
  //Serial.println("");


}

// ------------------------------------------

char* str2char(String command) {
  if (command.length() != 0) {
    char *p = const_cast<char*>(command.c_str());
    return p;
  }
}

