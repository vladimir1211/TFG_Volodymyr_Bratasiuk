

// Motor phases pins
//-----------------------------------------------------------------------------
#define HU  5   // High-side switch U
#define LU  6   // Low-side switch U
#define HV  7   // High-side switch V
#define LV  8   // Low-side switch V
#define HW  9   // High-side switch W
#define LW  10  // Low-side switch W
//-----------------------------------------------------------------------------

// Hall sensor pins
//-----------------------------------------------------------------------------
#define HALL_U 2  // Hall sensor phase U
#define HALL_V 3  // Hall sensor phase V
#define HALL_W 4  // Hall sensor phase W
//-----------------------------------------------------------------------------

// Pins for aditional control of TMC6140-LA
//-----------------------------------------------------------------------------
#define DRV_STR 11      //Set to 1: 0.5A / Set to 0: 1A
#define GAIN 12         //Set to 1: gain 50x / Set to 0: gain 20x
#define ENABLE_PIN 13   //Set to 1: enable / Set to 0: disable
#define SH_0 22         //Set to 1: short to ground / Set to 0: no short
#define SH_1 21         //Set to 1: short to ground / Set to 0: no short
//------------------------------------------------------------------------------

//
#define PP 15 // Pole pairs
//
#define LED 25 // LED pin

#include <SimpleFOC.h>


// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(HALL_U, HALL_V, HALL_W, PP);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// BLDC driver instance
//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair 
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
//  - enable pin    - (optional input)
BLDCDriver6PWM driver = BLDCDriver6PWM(HU, LU, HV, LV, HW, LW, ENABLE_PIN);
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(PP);

// velocity set point variable
float target_velocity = 5;
// include commander interface
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }


void receiveEvent(int bytes) {

  digitalWrite(LED, HIGH);
  

  int x = Wire.read();    // read one character from the I2C
  
  motor.move(x);

  Serial.print( x);
  Serial.print( "\n");
  digitalWrite(LED, LOW);
}
void seti2c() {
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  delay(500);
  // Start the I2C Bus as Slave on address 9
  Wire.begin(9); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
}

void mode0() {
  // Set the motor to mode 0
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 4;
  Serial.print("MODE 0 SET\n");
}

void mode1() {
  // Set the motor to mode 1
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 8;
  Serial.print("MODE 1 SET\n");
}

void mode2() {
  // Set the motor to mode 2
  motor.PID_velocity.P = 0.9;
  motor.PID_velocity.I = 2;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 10;
  Serial.print("MODE 2 SET\n");
}

void change_mode(char* cmd) {
  if (cmd[0] == '0') {
    mode0();
  } else if (cmd[0] == '1') {
    mode1();
  } else if (cmd[0] == '2') {
    mode2();
  }
}

void disable_enable(char* cmd) {
  if (cmd[0] == '0') {
    digitalWrite(SH_0, LOW);
    digitalWrite(SH_1, LOW);
    motor.linkSensor(nullptr);
    
    // motor.move(0);
    Serial.print("DISABLE\n");
  } else if (cmd[0] == '1') {
    digitalWrite(SH_0, HIGH);
    digitalWrite(SH_1, HIGH);
    motor.linkSensor(&sensor);
    Serial.print("ENABLE\n");
  }
}


void setup(){

seti2c();

pinMode(DRV_STR, OUTPUT);
pinMode(GAIN, OUTPUT);
pinMode(ENABLE_PIN, OUTPUT);
pinMode(SH_0, OUTPUT);
pinMode(SH_1, OUTPUT);

digitalWrite(ENABLE_PIN, HIGH);
digitalWrite(DRV_STR, LOW);
digitalWrite(GAIN, LOW);
digitalWrite(SH_0, HIGH);
digitalWrite(SH_1, HIGH);

// Serial configuration
//-----------------------------------------------------------------------------
Serial.begin(115200); // 115200 baud rate
delay(10);
Serial.print("Serial ready.\n");
//-----------------------------------------------------------------------------

// Hall sensor configuration
//-----------------------------------------------------------------------------
// check if you need internal pullups
sensor.pullup = Pullup::USE_EXTERN;
// initialise encoder hardware
sensor.init();
// hardware interrupt enable
sensor.enableInterrupts(doA, doB, doC);
Serial.print("Hall sensor configuration done.\n");
//------------------------------------------------------------------------------

// Driver configuration
//------------------------------------------------------------------------------
// power supply voltage [V]
driver.voltage_power_supply = 24.0;

driver.init();
// link the motor and the driver
motor.linkDriver(&driver);

motor.linkSensor(&sensor);



// set control loop type to be used
motor.controller = MotionControlType::velocity;

motor.torque_controller = TorqueControlType::voltage;

motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// velocity PI controller parameters
// default P=0.5 I = 10
motor.PID_velocity.P = 0.1;
motor.PID_velocity.I = 0.5;
motor.PID_velocity.D = 0;
motor.PID_velocity.output_ramp = 0;

motor.PID_velocity.limit = 20;
//default voltage_power_supply
motor.voltage_limit = 10;

// velocity low pass filtering
// default 5ms - try different values to see what is the best. 
// the lower the less filtered
motor.LPF_velocity.Tf = 0.3;
  

motor.useMonitoring(Serial);
motor.monitor_downsample = 100; // disable monitor at first - optional

// initialize motor
motor.init();
// align sensor and start FOC
//motor.initFOC(zero_electric_offset, sensor_direction);
motor.initFOC(2.09,Direction::CCW);
//------------------------------------------------------------------------------

command.add('M',doMotor,"target setting");

command.add('C', change_mode, "Change control mode");

command.add('D', disable_enable, "Disable motor-0 Enable motor-1");

// monitoring port
Serial.println("Motor ready.");
Serial.println("Set the target velocity using serial terminal:");
delay(10);

motor.move(2);

}
void loop(){
  
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();


  motor.monitor();

  // user communication
  command.run();


}