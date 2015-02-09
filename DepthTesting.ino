
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int GPSOn=9;  //This is the active high pin used to turn on the GPS module.
int GPSTx=7;  //This is the Tx pin for the GPS module.
int GPSRx=8;  //This is the Rx pin for the GPS module.
int Turb=A0;  //This is the pin for the turbidity sensor.
int Cond=A1;  //This is the pin for the conductivity sensor.
int Temp=A2;  //This is the pin for the temperature sensor.
int Depth=A3;  //This is the pin for the depth gauge.

SoftwareSerial GPSSerial(GPSTx, GPSRx);  //Set up the serial port for the GPS.
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean);

void setup(){
  Serial.begin(9600);  //Start serial communication at 9600 bps.
  pinMode(GPSOn, OUTPUT);  //Set up the GPS power pin as an output.
  digitalWrite(GPSOn, HIGH);  //Turn the GPS on.
  GPS.begin(9600);  //Start GPS serial communication at 9600 bps.
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop(){
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA())) 
          return;
  }
  delay(1000);
  Serial.print(GPS.year, DEC);
  Serial.print(GPS.month, DEC);
  Serial.print(GPS.day, DEC);
  Serial.print("T");
  Serial.print(GPS.hour, DEC);
  Serial.print(":");
  Serial.print(GPS.minute, DEC);
  Serial.print(":");
  Serial.print(GPS.seconds, DEC);
  Serial.print(".");
  Serial.print(GPS.milliseconds);
  Serial.print(",");
  Serial.print(GPS.latitude, 4);
  Serial.print(GPS.lat);
  Serial.print(",");
  Serial.print(GPS.longitude, 4);
  Serial.print(GPS.lon);
  Serial.print(",");
  Serial.print(analogRead(Turb));
  Serial.print(",");
  Serial.print(analogRead(Cond));
  Serial.print(",");
  Serial.print(analogRead(Temp));
  Serial.print(",");
  delay(200);
  Serial.print(analogRead(Depth));
  Serial.println();
}
