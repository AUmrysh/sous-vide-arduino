#include <TM1637.h>

#include <max6675.h>

#include <PID_v1.h>

#define ON 1
#define OFF 0

int8_t tempDisp[] = {0x00,0x00,0x00,0x00};
unsigned char ClockPoint = 1;
unsigned char Update;
unsigned char halfsecond = 0;
unsigned char second;
unsigned char minute = 0;
unsigned char hour = 12;

int pinA = 7;

int pinclk = 5;
int pincs = 4;
int pindo = 3;

double kp = 5;
double ki = 1;
double kd = 1;

#define CLK 9
#define DIO 10

int WindowSize = 2000;
unsigned long windowStartTime;

double Setpoint, Input, Output;

PID p(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_E, DIRECT);
TM1637 tm1637(CLK,DIO);

int ktcSO = 3;
int ktcCS = 4;
int ktcCLK = 5;
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

//float res;
void setup() {
  // put your setup code here, to run once:
  pinMode(pinA, OUTPUT);
  Serial.begin(9600);

  Setpoint = 60;

  p.SetMode(AUTOMATIC);

  windowStartTime = millis();
  p.SetOutputLimits(0, WindowSize);
  digitalWrite(pinA, LOW);

  tm1637.set();
  tm1637.init();

  delay(500);
}

void loop() {
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  
  //handle user input
  if (Serial.peek() != -1) {
    String userin = Serial.readString();
    String unit = userin.substring(0,1);
    String temp = userin.substring(1,userin.length());
    if (unit.equalsIgnoreCase("C")){
      //celsius
      int ctemp = temp.toInt();
      Setpoint = ctemp;
    }
    else if (unit.equalsIgnoreCase("F")){
      //fahrenheit
      int ftemp = temp.toInt();
      //lol convert to C and set to that
      Setpoint = ((ftemp - 32) / 1.8);
    }
    //otherwise do nothing because this is an invalid command... maybe clear the input buffer and respond with an error
    
  }

  double temperature = ktc.readCelsius();
  if (isnan(temperature)) {
    Serial.print("\n\nERROR: unable to read thermocouple. Check the pinout of the connector and ensure that it is connected.");
  }
  else {
    Serial.print("\nTemperature: ");
    Serial.print(temperature);
    Serial.print("/");
    Serial.println(Setpoint);
  
  Input = temperature;
  p.Compute();

  //update display
    //update temp display
  int t = int(temperature);
    //update temp display
  tempDisp[3] = t % 10;
  tempDisp[2] = (t/10) % 10;
  tempDisp[1] = (t/100) % 10;
  tempDisp[0] = (t/1000) % 10;
  tm1637.display(tempDisp);

  //if (millis() % 1000 == 0) {
//    Serial.print("output: ");
//    Serial.println(Output);
//    Serial.println(millis() - windowStartTime);
  //}

  if(isnan(Output)) {
    //error with PID?
    digitalWrite(pinA, LOW);
  }
    else {
      if (millis() - windowStartTime > WindowSize) {
        windowStartTime += WindowSize;
      }
      if (Output < millis() - windowStartTime) {
        //turn relay off
        digitalWrite(pinA, LOW);
      }
      else {
        //turn relay on
        digitalWrite(pinA, HIGH);
      }
    }
  }

  tm1637.point(POINT_OFF);
  
  delay(500);
}
