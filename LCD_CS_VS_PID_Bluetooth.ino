// This code combines LCD, VS, CS, PID, Bluetooth

// Read current and voltage sensors
// voltage sensor is a voltage divider(1M + 91k)
// due to tolerance, 1M is actually ~948.7k and 91k is 91.29
// at max voltage(55 V) , arduino gets 55*91.29/(91.29+948.7)= 4.82788 V to its analog input

/*
Our LCD's datasheet: http://tronixstuff.com/wp-content/uploads/2010/07/2004-lcd-data-sheet.pdf  
 Pinout:
  LCD  |  Arduino
 ------|-----------
  1    |  GND
  2    |  +5V
  3    |  output of 10k pot (contrast control)
  4    |  pin 9
  5    |  GND
  6    |  pin 8
  7-10 |  we don't need these for our implementation
  11   |  pin 5
  12   |  pin 4
  13   |  pin 3
  l4   |  pin 2
  15-16|  we don't need these either
 */
 /* Arduino pins used:
   LCD: Digital pins 9, 8, 5, 4, 3, 2  - pins 6&7 left open for Julian's bluetooth
   Current Sensor: A0
   Voltage Sensor: A1
 */
 
//Libraries
#include <LiquidCrystal.h>
#include <string.h>
#include <SoftwareSerial.h>   //Software Serial Port
#define RxD 6                 //Bluetooth recieve pin =6
#define TxD 7                 //Bluetooth transmit pin =7

#define DEBUG_ENABLED  1

SoftwareSerial blueToothSerial(RxD,TxD);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(9, 8, 5, 4, 3, 2);

int iSensorPin = A2;
int iRawValue;
float I_bat;
float VoutFromCS; // Vout from current Sensor
float Vcc = 4.658; // the actual Vcc that Arduino outputs
int nBitsADC = 10; // our ADC has 10bits
float samplesI[200];
int i;

int vSensorPin = A1;
int vRawValue;
float V;
float VoutFromVS; // Vout from voltage Sensor
float samplesV[200];

const int throttlePin = A0;    //manual throttle input pin
const int throttleOut = 11;   //throttle voltage to the motor controller (need RC filter w/ C ~1mF)
const int ledPin = 12;        //LED that signals whether cruise control is on or off
const int hallread = 4;        // hall effect sensor output
int refspeed;                  //desired speed (rpm), specified by user
int throttleIn = 0;           //variable that stores throttle input value 
unsigned long duration;       //variable that stores pulse time (ms)
unsigned long rpm;            // variable that stores actual speed (rpm) 
const float kp = 0.05;       //kp parameter for PID controller (tuned)
const float ki = 0.05;       //ki parameter for PID controller (tuned)
long control;                //voltage signal sent to motor
unsigned long t0 = 0;        //start of time interval
unsigned long t1 = 0;        //end of time interval
volatile boolean cruise = false;      //flag for turning cruise control on/off
//volatile boolean deactivatecruise = true;  //flag for determining if throttle deactivates cruise control
const unsigned int DEBOUNCE_TIME = 300;
static unsigned long last_interrupt_time = 0;

////// PID parameters initialization //////
long error;                  // error between user's ref speed and actual speed  
float integral = 0;          //integral for PID controller
float P;                     //proportional controller value
float I;                     //integral controller value
unsigned int dt;             //change in time (for integral purposes)
const int upperlimit = 50;   //integral limit to prevent windup
const int lowerlimit = -50;  //integral limit to prevent windup 

////////////////////////////////////////////////////////////////////////////////////////
//////////                           Setup                         /////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  i =0;
  lcd.begin(20, 4); // our display has 20 columns and 4 rows 
  lcd.setCursor(0,0); // set cursor to 3rd row of LCD
  lcd.print("Group 2B");
  
  pinMode(throttleOut, OUTPUT);
  pinMode(hallread, INPUT);     //read hall effect output
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  pinMode(ledPin, OUTPUT);
  setupBlueToothConnection();
  attachInterrupt(0, cruisecontrol, CHANGE);  
}

void loop()
{
    //Read Analogs 
    duration = pulseIn(hallread, HIGH);  // hall effect output pulse time (ms)
    rpm = 370000/duration;    // actual speed (rpm) of motor; gain determined by comparing to dyno speed reading
    //rpm = 127;  //testing purposes 
  
    throttleIn = map(analogRead(throttlePin), 0, 1023, 0, 255);  //read the throttle voltage and constrain to 0-255

    //Vehicle Control Logic
    if(throttleIn > 50) // && deactivatecruise == true){      //User is manually controlling speed
      {
      control = throttleIn;
      Serial.println(control);
      blueToothSerial.println(control);
      cruise = false;
    }      
    else if(blueToothSerial.available()||Serial.available()){ //User sends a speed reference from either bluetooth or serial window
      serialSpeed(); //Update reference speed                                          
    }    
    else if(cruise == true){     //Cruise Control is ON
      PID(); //adjust output based on PID
      //Serial.println(rpm);             //debug
      //blueToothSerial.println(rpm);    //debug      
    }
    if (cruise == false) {digitalWrite(ledPin, LOW);}
    else if (cruise == true) {digitalWrite(ledPin, HIGH);}
   
    //Vehicle Control Voltage
    analogWrite(throttleOut,control);    //send PWM to the motor  
    //t0 = t1;  //save time value for next loop dt calculation
    //if(throttleIn < 50) {deactivatecruise == true;}    //throttle has returned to zero, therefore it's now able to deactiviate cruise control
    
    measureVI();
}

////////////////////////////////////////////////////////////////////////////////////////
//////////                       FUNCTIONS                         /////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void measureVI(void){
  //Ivan's code:
    iRawValue = analogRead(iSensorPin) - 11; // the sensor is giving slightly higher ADC value
    vRawValue = analogRead(vSensorPin); 
  
    VoutFromCS = (float) (iRawValue/(pow(2,nBitsADC) - 1))*Vcc;
    VoutFromVS = (float) (vRawValue/(pow(2,nBitsADC) - 1))*Vcc;

    samplesI[i] = (float) 73.3 * (VoutFromCS/Vcc) -36.7; // from ACS711EX data sheet
                                             // http://www.pololu.com/product/2453
    samplesV[i] = (float) (VoutFromVS/4.807877) * 55; // at 55 V, A0 will get 4.807877 V
    
  
    if(i == 199){ // algorithm that finds average measured value to minimize oscillations
      for(int j=0; j<200; j++){
        I_bat += samplesI[j];
        V += samplesV[j];
      }
      I_bat = I_bat/200;
      V = V/200;
      i =0;
    
      lcd.setCursor(0,2); // set cursor to 3rd row of LCD
      lcd.print("Current: ");
      lcd.print(I_bat);
      lcd.print(" A   ");
      
      lcd.setCursor(0,3); // set cursor to 3rd row of LCD
      lcd.print("Voltage: ");
      lcd.print(V);
      lcd.print(" V   ");
      
      delay(100);
     }
      else i++;
}

void serialSpeed()    //This function takes in a value from the serial/BT command window
{                     //and updates/prints the speed value on both serial and bt terminals
//  Serial.println();
//  blueToothSerial.println();

  while (blueToothSerial.available() > 0){      //while there is BT data available
      refspeed = blueToothSerial.parseInt();       //pull out the first string of numbers
      refspeed = constrain(refspeed, 0, 255);         //constrain it to a value of 0-240 (max rpm)
//      Serial.println(refspeed);                    //print it on the serial monitor so we know
//      blueToothSerial.println(refspeed);          //print it on the BT monitor so we know
      blueToothSerial.flush();
  }

  while (Serial.available() > 0){              //same as the BT loop but for serial command
      refspeed = Serial.parseInt();            //read incoming reference speed
      refspeed = constrain(refspeed, 0, 255);  //constrain it to a value of 0-240 (max rpm)
//      Serial.println(refspeed);
//      blueToothSerial.println(refspeed);
        blueToothSerial.flush();
  }

}

/////////// PID Function /////////////
void PID(){
    t1 = millis();  // start of time interval
    dt = t1 - t0;  //
    error = refspeed - rpm;      // error between user's ref speed and actual speed
    integral = integral + error*dt*0.001; // x0.001 to get dt into sec
    
    if (integral > upperlimit) {integral = upperlimit;}     //windup prevention
    else if (integral < lowerlimit) {integral = lowerlimit;}    //windup prevention
     
    P = error*kp;
    I = integral*ki;
    control = 51*(P+I);  // voltage signal to motor, 51 b/c 255/5V
    control = constrain(control, 0, 190); // constrain voltage input to motor
    t0 = t1;
    
//    Serial.print("Refspeed: "); Serial.println(refspeed);
//    Serial.print("Control: "); Serial.println(control);
//    Serial.print("Error: "); Serial.println(error);
//    Serial.print("Control: "); Serial.println(control);
//    Serial.print("P: ");
//    Serial.println(P);
//    Serial.print("I: ");
//    Serial.println(I);
//    Serial.print("Cruise: ");
//    Serial.println(cruise);
}

/////////// Set Cruise Control Fuction ///////////
void cruisecontrol(){
  unsigned long interrupt_time= millis();
  if(interrupt_time - last_interrupt_time > DEBOUNCE_TIME) {
    cruise = !cruise;
   }
  else {
    refspeed = rpm;
 //   deactivatecruise = false;
  }
    last_interrupt_time = interrupt_time;
}

/////////// Bluetooth Setup ///////////
void setupBlueToothConnection()
{
  blueToothSerial.begin(38400); //Set BluetoothBee BaudRate to default baud rate 38400
  blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
  blueToothSerial.print("\r\n+STNA=SeeedBTSlave\r\n"); //set the bluetooth name as "SeeedBTSlave"
  blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
  blueToothSerial.print("\r\n+STAUTO=0\r\n"); // Auto-connection should be forbidden here
  delay(2000); // This delay is required.
  blueToothSerial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable 
  Serial.println("The slave bluetooth is inquirable!");
  delay(2000); // This delay is required.
  blueToothSerial.flush();
}
