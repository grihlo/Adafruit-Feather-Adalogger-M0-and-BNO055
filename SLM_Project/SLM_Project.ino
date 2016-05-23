#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

unsigned long time;
unsigned long timer;

unsigned long dinterval;

const int buttonPin = 6;
int buttonState = 0;

// Set the pins used
#define cardSelect 4

File logfile;

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup() {

  pinMode(buttonPin, INPUT);
  Serial.begin(115200);
  
   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  

  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
}

uint8_t i=0;
void loop() {
  digitalWrite(8, HIGH);
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH)
  {
    char filename[15];
    strcpy(filename, "ANALOG00.TXT");
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = '0' + i/10;
      filename[7] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) {
        break;
      }
    }
  
    logfile = SD.open(filename, FILE_WRITE);
    if( ! logfile ) {
      Serial.print("Couldnt create "); 
      Serial.println(filename);
      error(3);
    }
    
    digitalWrite(8, LOW);
    delay(2000);
    
    for(int i = 0; i < 800; i++)
    {
      digitalWrite(8, HIGH);
      
      time = millis();
      
      imu::Vector<3> accelm = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu::Quaternion quat = bno.getQuat();
      imu::Vector<3> vec = quat.toEuler();
      
      logfile.print(time);
      logfile.print(" ");
      
      logfile.print(accelm.x());
      logfile.print(" ");
      logfile.print(accelm.y());
      logfile.print(" ");

      logfile.println(vec[1]);
      
      timer = millis();

      dinterval = (10 + time) - timer;
      if(dinterval > 10)
      {
        dinterval = 0;
      }
      delay(dinterval);
    }
    logfile.close();
    
    digitalWrite(8, LOW);
    delay(2000);
  }
  
  delay(10);
}
