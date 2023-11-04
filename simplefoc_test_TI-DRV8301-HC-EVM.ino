#include <SimpleFOC.h>

//nominal supply voltage
#define suppVoltage 30
#define maxVoltage 32

#define pot_pin PE_10

//motor parameters
#define pole_pairs 20
#define KV_rating 10.2

//motor PWM outputs
#define pwm_A PC_7
#define pwm_B PB_5
#define pwm_C PB_3

//DRV8301 specific pins
#define EN_GATE PF_13
#define M_PWM PE_9 
#define M_OC PE_11
#define OC_ADJ PF_14

//hall sensor pins
#define hall_A PE_13
#define hall_B PF_15
#define hall_C PG_14

//current sense pins
#define DC_CAL PD_14
#define IA_FB PA_5
#define IB_FB PA_6
#define IC_FB PA_7

#define CurrGain 21.65
#define R_shunt 0.002

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
BLDCMotor motor = BLDCMotor(pole_pairs, KV_rating);

// Hall sensor instance
// HallSensor(int hallA, int hallB , int hallC , int pp)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
HallSensor sensor = HallSensor(hall_A, hall_B, hall_C, pole_pairs, EN_GATE);

// Interrupt routine initialization
// channel A and B callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(pwm_A, pwm_B, pwm_C);

// instantiate sensor 

//  LowsideCurrentSense(shunt_resistance, gain, adc_a, adc_b, adc_c)
LowsideCurrentSense current_sense = LowsideCurrentSense(R_shunt, CurrGain, IA_FB, IB_FB, IC_FB);

//instantiate commander
Commander commander = Commander(Serial);
void doMotor(char* cmd){commander.motor(&motor, cmd);}

// potentiometer value
int pot_val;

void setup() {
  // initialize potentiometer input pin
  pinMode(pot_pin, INPUT_ANALOG);
  // initialize sensor hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = suppVoltage;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = maxVoltage;
  // driver init
  driver.init();

  // link the driver with the current sense
  current_sense.linkDriver(&driver);
  // init current sense
  current_sense.init();

   // use monitoring with the BLDCMotor
  Serial.begin(115200);
  // monitoring port
  motor.useMonitoring(Serial);

  // subscribe motor to the commands
  commander.add('M',doMotor,"motor");

  // init sensor
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // init driver
  // link the motor to the driver
  motor.linkDriver(&driver);
  // link driver and the current sense
  
  // link the motor to current sense
  motor.linkCurrentSense(&current_sense);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // initialize motor
  motor.init();


  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  // read potentiometer
  pot_val = analogRead(pot_pin);
  pot_val = pot_val*0.002;
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // setting the target velocity or 2rad/s
  motor.move(pot_val);

  // monitoring function outputting motor variables to the serial terminal 
  motor.monitor();

  // read user commands
  commander.run();
}
