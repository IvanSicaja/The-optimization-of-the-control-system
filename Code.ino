//kod za regulaciju nagiba grede
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define PIND_A 2
#define PIND_B 3
#define ENC_PIN_A 2 //pin kanala A enkodera
#define ENC_PIN_B 3 //pin kanala B enkodera
#define READ_ENC_A bitRead(PIND, PIND_A)
#define READ_ENC_B bitRead(PIND, PIND_B)

//ukljucivanje bibilioteka potrebnih za rad sa senzorom BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define MOTORB_1A 5
#define MOTORB_1B 6



//stvaranje instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float kut_BNO;



volatile long encBrojac = 0;//###
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//##################################################################

void encISR()
{
  if (READ_ENC_A == HIGH)//kanal A L->H (rastuci brid)
    READ_ENC_B == HIGH ? encBrojac++ : encBrojac--;
  else //kanal A H->L (padajuci brid)
    READ_ENC_B == HIGH ? encBrojac-- : encBrojac++;
}

void motorSmjer1(int setSpeed)
{
  digitalWrite(MOTORB_1A, LOW);
  analogWrite(MOTORB_1B, setSpeed);
}
void motorSmjer2(int setSpeed)
{
  analogWrite(MOTORB_1A, setSpeed);
  digitalWrite(MOTORB_1B, LOW);
}
void motorStop()
{
  digitalWrite(MOTORB_1A, LOW);
  digitalWrite(MOTORB_1B, LOW);
}
//##################################################################

void setup()
{
  Serial.begin(9600);
    //Serial.println("Zeljeni_Kut,Kut_Enkodera");
  Serial.println("Zeljeni_Kut,Kut_Enkodera,Pogreska,P_DIO,I_DIO,D_DIO,PIDSignal");
 
  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);

  if (!bno.begin()) //ako je doslo do problema s povezivanjem s BNO055
  {
    Serial.print("Problem s povezivanjem sa senzorom BNO055!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), encISR, CHANGE);


  pinMode(MOTORB_1A, OUTPUT);
  pinMode(MOTORB_1B, OUTPUT);
  digitalWrite(MOTORB_1A, LOW);
  digitalWrite(MOTORB_1B, LOW);


}

void loop() {

  //dohvati polozaj
  sensors_event_t eventT;
  bno.getEvent(&eventT);
  kut_BNO = eventT.orientation.z;
  kut_BNO < 0 ? kut_BNO += 360 : kut_BNO;

  int target = kut_BNO;
  //int target = 120;


  // tu ide algoritam regulatora

  // PID constants
  float kp = 3.5;
  float kd = 0.5;
  float ki = 1.5;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int kut = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    kut = encBrojac / (4.6);
  }

  // error
  int e = kut - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;
  

  /*
    Serial.print("PID TOTAL BEFOR MAP:  ");
    Serial.print(PID_total);
    Serial.print(",");
    Serial.print(PID_p);
    Serial.print(",");
    Serial.print(PID_i);
    Serial.print(",");
    Serial.print(PID_d);
    Serial.print(".");
  */


  /*
    Serial.print(" PID TOTAL AFTER MAP:  ");
    Serial.print(PID_total);
  */
  if (u < 0) {
    u = u*(-1);
  }
  if (u > 205) {
    u = 205;
  }


  // store previous error
  eprev = e;


  //gotov algoritam regulatora


  if ((kut - target) > 0)
    motorSmjer1(u + 50);

  if ((kut - target) < -0)
    motorSmjer2(u + 50);

  if (abs(kut - target) == 1)
    motorStop();
    
    
  
  Serial.print(target);
  Serial.print(" ");
  Serial.println(kut);
  
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  Serial.print(kp);
  Serial.print(" ");
  Serial.print(ki);
  Serial.print(" ");
  Serial.print(kd);
  Serial.print(" ");
  Serial.print(u);
  Serial.print(" ");
  Serial.println(180);
  

  //ispis stanja
  //Serial.print(millis());
  //Serial.print("  ");
  //Serial.print(encBrojac);

  /*
    Serial.println("  ");
    Serial.print("Kut ziroskopa po z osi: ");
    Serial.println(kut_BNO);
    Serial.print("Kut enkodera: ");
    Serial.println(kut);
    Serial.print("RAZLIKA KUTEVA Ž-E: ");
    Serial.println(kut_BNO-kut);
  */

  /*
    Serial.print(" PID TOTAL:  ");
    Serial.print(PID_total);
    Serial.print(" P:  ");
    Serial.print(PID_p);
    Serial.print(" D:  ");
    Serial.print(PID_d);
    Serial.print(" I:  ");
    Serial.println(PID_i);
  */

  //delay(1000);




  //   digitalWrite(MOTORB_1A, LOW);
  // analogWrite(MOTORB_1B, 160);

}
