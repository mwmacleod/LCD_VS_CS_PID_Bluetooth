//Libraries
#include <SoftwareSerial.h>   //Software Serial Port
#define RxD 6                 //Bluetooth recieve pin =6
#define TxD 7                 //Bluetooth transmit pin =7
 
#define DEBUG_ENABLED  1
 
SoftwareSerial blueToothSerial(RxD,TxD);

int buttonInt = 0;  // Pin 2, button interrupt
int brakeInt = 1;   // Pin 3, brake interrupt

const int throttlePin = A0;    //manual throttle input pin
const int buttonPin = 2;      //push button that turns on cruise control
const int brakePin = 3;       //push button that simulates hitting the brakes
const int ledPin = 12;        //LED that signals whether cruise control is on or off
const int throttleOut = 11;   //throttle voltage to the motor controller (need RC filter w/ C ~1mF)
const int hallread = 4;        // hall effect sensor output
volatile int refspeed;         //desired speed (rpm), specified by user
int throttleIn = 0;           //variable that stores throttle input value 
unsigned long duration;       //variable that stores pulse time (ms)
unsigned long rpm;            // variable that stores actual speed (rpm)

// PID parameters //
long error;        // error between user's ref speed and actual speed
long lasterror;  
float integral = 0;         //integral for PID controller
const float kp = 0.05;      //kp parameter for PID controller (tuned)
const float ki = 0.05;      //ki parameter for PID controller (tuned)
const float kd = 0.05;
float P;                    //proportional controller value
float I;                    //integral controller value
float D;                    
long control;              //voltage signal sent to motor
unsigned long t0 = 0;        //start of time interval
unsigned long t1 = 0;        //end of time interval
unsigned int dt;               //change in time (for integral purposes)
const int upperlimit = 45;   //integral limit to prevent windup
const int lowerlimit = -45;   //integral limit to prevent windup

volatile boolean cruise = false;      //flag for turning cruise control on/off
//volatile boolean braking = false;      //flag for turning cruise control on/off via hitting the brakes
const unsigned int DEBOUNCE_TIME = 500;
static unsigned long last_interrupt_time = 0;
static unsigned long last_interrupt_time2 = 0;

////////////////////////////////////////////////////////////////////////////////////////
//////////                           Setup                         /////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void setup()
{ 
  Serial.begin(9600);
  pinMode(throttleOut, OUTPUT);
  pinMode(hallread, INPUT);     //read hall effect output
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(brakePin, INPUT);
  setupBlueToothConnection();  
  attachInterrupt(buttonInt, cruisecontrol, CHANGE);  //interrupt for turning cruise control on/off
  attachInterrupt(brakeInt, braking, CHANGE);      //interrupt for simulating hitting the brakes
}


////////////////////////////////////////////////////////////////////////////////////////
//////////                       Main code                         /////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  //This loop takes in a speed from the bluetooth and/or serial command windows
  //that speed is then converted to a PWM signal and sent to the motor
    while(true)
  { 
      //read motor speed 
      duration = pulseIn(hallread, HIGH);  // hall effect output pulse time (ms)
      rpm = 370000/duration;    // actual speed (rpm) of motor; gain determined by comparing to dyno speed reading
  
      //read the throttle voltage and constrain to 0-255
      throttleIn = map(analogRead(throttlePin), 0, 1023, 0, 255);
      throttleIn = constrain(throttleIn, 0, 180);
          
      if(blueToothSerial.available()||Serial.available()){ //Check for a speed command from either bluetooth or serial window
        serialSpeed();
      }
      
      if (cruise == false) // manual throttle mode
      { 
        digitalWrite(ledPin, LOW);
        //Serial.println();
        Serial.print("[MANUAL] Speed (rpm): ");
        //Serial.print();
        Serial.print(rpm);
        Serial.print(" Cruise? ");
        Serial.println(cruise);
        analogWrite(throttleOut,throttleIn);    //send PWM to the motor
      } 
      else
      {
        digitalWrite(ledPin, HIGH);
        PID();
        if (throttleIn > control) {              // if throttle > cruise control speed, temporarily go into manual mode
          analogWrite(throttleOut, throttleIn);
          }
        else{
          analogWrite(throttleOut,control);
        
        // debugging purposes
        Serial.print("[CRUISE CONTROL]");
        Serial.print(" rpm:"); Serial.print(rpm);
        Serial.print(" error:"); Serial.print(error); 
        Serial.print(" dt:"); Serial.print(dt); 
        Serial.print(" P:"); Serial.print(P);
        Serial.print(" I:"); Serial.print(I);  
        Serial.print(" D:"); Serial.println(D);
        } 
       }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
//////////                       FUNCTIONS                         /////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void serialSpeed()    //This function takes in a value from the serial/BT command window
{                     //and updates/prints the speed value on both serial and bt terminals

Serial.println();
Serial.print("[SERIAL] Setting speed to: ");
blueToothSerial.print("[SERIAL] Setting speed to: ");
Serial.println();
  
  while (blueToothSerial.available() > 0){      //while there is BT data available
      refspeed = blueToothSerial.parseInt();       //pull out the first string of numbers
      refspeed = constrain(refspeed, 0, 240);         //constrain it to a value of 0-240 (max rpm)
      //Serial.println(refspeed);                    //print it on the serial monitor so we know
      blueToothSerial.println(refspeed);          //print it on the BT monitor so we know
  }
  
  while (Serial.available() > 0){              //same as the BT loop but for serial command
      refspeed = Serial.parseInt();            //read incoming reference speed
      refspeed = constrain(refspeed, 0, 240);  //constrain it to a value of 0-240 (max rpm)
      //Serial.println(refspeed);
      blueToothSerial.println(refspeed);
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
    D = kd*(error-lasterror);
    control = 51*(P+I+D);  // voltage signal to motor, 51 b/c 255/5V
    control = constrain(control, 0, 185); // constrain voltage input to motor
    t0 = t1;
    lasterror = error;
}

/////////// Set Cruise Control Fuction ///////////
void cruisecontrol(){
  unsigned long interrupt_time= millis();
  if(interrupt_time - last_interrupt_time > DEBOUNCE_TIME) {
    cruise = !cruise;
    refspeed = rpm;
  }
    last_interrupt_time = interrupt_time;
}

/////////// Braking Fuction ///////////
void braking(){
  unsigned long interrupt_time2= millis();
  if(interrupt_time2 - last_interrupt_time2 > DEBOUNCE_TIME) {
    cruise = false;      // turn off cruise control
  }
    last_interrupt_time2 = interrupt_time2;
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



