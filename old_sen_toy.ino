#include <SFE_BMP180.h>
#include <Wire.h>
#include "pitches.h"
#include <SoftwareSerial.h>
#define ALTITUDE 91
// You will need to create an SFE_BMP180 object, here called "pressure":

SFE_BMP180 pressure;

SoftwareSerial gpsSerial(10, 11); // RX, TX (TX not used)
const int sentenceSize = 80;

char sentence[sentenceSize];

int SHT_clockPin = 5;  // pin used for clock
int SHT_dataPin  = 4;  // pin used for data

float test1;
float test2;


int melody[] = {
  NOTE_AS6,NOTE_AS6, NOTE_AS6, NOTE_AS6, NOTE_AS6, NOTE_AS6};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  2,2,2,4,4,4};

float bot = -50;
float top = 4.4444;

int hasWrite = 0;

int red = 9;   // Red LED,   connected to digital pin 13
int grn = 6;  // Green LED, connected to digital pin 11
int blu = 13;  // Blue LED,  connected to digital pin 10

int redSing = 12;
int grnSing = 7;

// Program variables to send to the pins
int redVal = 0;   
int grnVal = 0;
int bluVal = 0;

int potPin = 5; // Potentiometer output connected to analog pin 0
int potVal = 0;

void setup(){
  Serial.begin(9600); // open serial at 9600 bps
  gpsSerial.begin(9600);
  Serial.println("YES");
   pinMode(red, OUTPUT);   
  pinMode(grn, OUTPUT);   
  pinMode(blu, OUTPUT); 
  pinMode(grnSing, OUTPUT);
  pinMode(redSing, OUTPUT);
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
  while(1){
    Serial.println("BMP180 init fail\n\n");}
  }
}

void loop()
{
  
  static int i = 0;
  int test3 = 0;
  while(hasWrite == 0){
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    if (ch != '\n' && i < sentenceSize)
    {
      sentence[i] = ch;
      i++;
    }
    else
    {
     Serial.write(1); 
     sentence[i] = '\0';
     i = 0;
     displayGPS();
     test3 = 1;
    }
  }
  
  }
    float temperature = getTemperature();
  int sensorValue = analogRead(A0);
    potVal = analogRead(potPin);  

  if (potVal < 299) 
  {                  
    redVal = 0;  
    grnVal = 0;       
    bluVal = 255; 
    bot = -60;
   top = 40;
   
  }
  //green
  else if (potVal < 400)  
  {                  
    redVal = 0; 
    grnVal = 255;        
    bluVal = 0; 
    bot = 40.1;
    top = 60; 
  }
  //yellow
  
  //orange
   else if (potVal < 950) 
  {                  
    redVal = 255;
    grnVal = 160;  
    bluVal = 0;
    bot = 60.1;
    top = 80;
    
  }
  //red
   else  
  {                  
    redVal = 255;  
    grnVal = 0;      
    bluVal = 0; 
    bot = 80.1;
    top = 120;   
  }
  setColor(redVal,grnVal,bluVal);
  
  
  if(temperature > bot && temperature < top)
  {
    digitalWrite(redSing, LOW);
    digitalWrite(grnSing, HIGH);
  }
  else
  {
    digitalWrite(redSing, HIGH);
    digitalWrite(grnSing, LOW);
  }
   char status;
  double T,P,p0,a;
  
  
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
     // Serial.print("temperature: ");
      //Serial.print(T,2);
      //Serial.print(" deg C, ");
      //Serial.print((9.0/5.0)*T+32.0,2);
      //Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {


          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
         /* Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");*/
          
          
          
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  
  
  
  
  float humidity = getHumidity();
  if(humidity > 50)
  {
     for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    tone(8, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
  }
  
  Serial.println((9.0/5.0)*T+32.0);
  Serial.println( humidity);
  Serial.println(p0*0.0295333727);
  
  delay(1000);
  hasWrite = 0;
}



void setColor(int red1, int green1, int blue1)
{
  analogWrite(red, red1);
  analogWrite(grn, green1);
  analogWrite(blu, blue1);  
}

float getTemperature(){
  //Return Temperature in Celsius
  SHT_sendCommand(B00000011, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);

  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return (float)val * 0.01 - 40; //convert to celsius
}

float getHumidity(){
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);
  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return -4.0 + 0.0405 * val + -0.0000028 * val * val; 
}


void SHT_sendCommand(int command, int dataPin, int clockPin){
  // send a command to the SHTx sensor
  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);

  if (digitalRead(dataPin)) Serial.println("ACK error 0");
  digitalWrite(clockPin, LOW);
  if (!digitalRead(dataPin)) Serial.println("ACK error 1");
}


void SHT_waitForResult(int dataPin){
  // wait for the SHTx answer
  pinMode(dataPin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i){
    delay(2);
    ack = digitalRead(dataPin);
    if (ack == LOW) break;
  }

  if (ack == HIGH) Serial.println("ACK error 2");
}

int SHT_getData(int dataPin, int clockPin){
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

void SHT_skipCrc(int dataPin, int clockPin){
  // skip CRC data from the SHTx sensor
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}

void displayGPS()
{
  //Serial.write("not in loop"); 
  char field[20];
  getField(field, 0);
 // if (strcmp(field, "$GPRMC") == 0)
String field2 = "";
 if (strcmp(field,"$GPRMC") == 0)
  {
    //Serial.print("Lat: ");
    getField(field, 3);  // number
    field2 = field2 + (field);
    getField(field, 4); // N/S
    field2 = field2 + (field) + " ";
    
   // Serial.print(" Long: ");
    getField(field, 5);  // number
    field2 = field2 + (field);
    getField(field, 6);  // E/W
    field2 = field2 + (field);
    
    Serial.println(field2);
    field2 = "";
    
    hasWrite = 1;

  }

}

void getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
} 

