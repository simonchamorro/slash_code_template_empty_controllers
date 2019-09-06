//=========================HEADER=============================================================
// Firmware for the Arduino managing the propulsion of the slash platform (UdeS Racecar)
//============================================================================================

/////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include <SPI.h>
#include <Servo.h> 
#define USB_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

///////////////////////////////////////////////////////////////////
// Init I/O
///////////////////////////////////////////////////////////////////

// Servo objects for PWM control of
// Sterring servo-motor
Servo steeringServo;

// ROS
ros::NodeHandle  nodeHandle;

//Publisher 
const int prop_sensors_msg_length = 8;
float prop_sensors_data[ prop_sensors_msg_length ];
std_msgs::Float32MultiArray prop_sensors_msg;
ros::Publisher prop_sensors_pub("prop_sensors", &prop_sensors_msg);

// Serial Communication
const int baud_rate = 57600;

// Slave Select pins for the encoder
const int slaveSelectEnc = 45;

// Pins for outputs PWM
const int ser_pin = 9;      // Servo 

// Custom drive
const int dri_pwm_pin     = 6 ;  // H bridge drive pwm
const int dri_dir_pin     = 42; //

///////////////////////////////////////////////////////////////////
// Parameters
///////////////////////////////////////////////////////////////////

// Controller

// TODO : VOUS DEVEZ DETEMINEZ DE BONS PARAMETRES SUIVANTS
const float filter_rc  =  0.5;
const float vel_kp     =  1.0;
const float vel_ki     =  0.0;
const float vel_kd     =  0.0;
const float pos_kp     =  1.0;
const float pos_kd     =  0.0;
const float pos_ki     =  0.0; 
const float pos_ei_sat =  10000.0; 
// FIN DU TODO

// Loop period 
const unsigned long time_period_low   = 2;  // 500 Hz for internal PID loop
const unsigned long time_period_high  = 10; // 100 Hz  for ROS communication

// Hardware min-zero-max range for the steering servo and the drive
const int pwm_min_ser = 30  ;
const int pwm_zer_ser = 90  ;
const int pwm_max_ser = 150 ;
const int pwm_min_dri = -511;
const int pwm_zer_dri = 0;
const int pwm_max_dri = 511;

const int dri_wakeup_time = 20; // micro second

// Units Conversion
const double batteryV  = 8;
const double maxAngle  = 40*(2*3.1416)/360;    //max steering angle in rad
const double rad2pwm   = (pwm_zer_ser-pwm_min_ser)/maxAngle;
const double volt2pwm  = (pwm_zer_dri-pwm_min_dri)/batteryV;
const double tick2m    = 0.00000323766; // To confirm

///////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////

// Inputs
float ser_ref    = 0; //rad
float dri_ref    = 0; //volt
int ctl_mode     = 0; // discrete control mode
int dri_standby  = 0; 

// Ouputs
int ser_pwm   = 0;
int   dri_pwm = 0;
float dri_cmd = 0;

// Controller memory (differentiation, filters and integral actions)
signed long enc_now   = 0;
signed long enc_old   = 0;

float pos_now   = 0;
float vel_now   = 0;
float vel_old   = 0;

float vel_error_int = 0 ;
float pos_error_int = 0;

// Loop timing
unsigned long time_now       = 0;
unsigned long time_last_low  = 0;
unsigned long time_last_high = 0;


///////////////////////////////////////////////////////////////////
// Encoder init/read/reset functions
///////////////////////////////////////////////////////////////////

//////////////////////////////////////
void initEncoder() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc, OUTPUT);
  
  // Raise select pin
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc,HIGH);       // Terminate SPI conversation 
}

//////////////////////////////////////
long readEncoder() {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  
  
  // Read encoder
  digitalWrite(slaveSelectEnc,LOW);       // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(slaveSelectEnc,HIGH);      // Terminate SPI conversation 
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

//////////////////////////////////////
void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc,HIGH);     // Terminate SPI conversation   
}


///////////////////////////////////////////////////////////////////
// Convertion functions
///////////////////////////////////////////////////////////////////

// Convertion function : Servo Angle --> PWM
double ser2pwm (double cmd) {
  
  // Scale and offset
  double pwm_d = cmd * rad2pwm + (double) pwm_zer_ser;
  
  // Rounding and conversion
  int pwm = (int) ( pwm_d + 0.5 );
  
  // Saturations
  if (pwm > pwm_max_ser) {
    pwm = pwm_max_ser;
  }
  if (pwm < pwm_min_ser) { 
    pwm = pwm_min_ser;
  }
  
  return pwm;
}

// Convertion function : Volt Command --> PWM
double cmd2pwm (double cmd) {

  int pwm = (int) ( cmd / batteryV * pwm_max_dri  + 0.5 );
  
  // Saturations
  if (pwm < pwm_min_dri) { 
    pwm = pwm_min_dri;
  }
  if (pwm > pwm_max_dri) {
    pwm = pwm_max_dri;
  }  
  
  return pwm;
}


///////////////////////////////////////////////////////////////////
// Set PWM value
///////////////////////////////////////////////////////////////////
void set_pwm( int pwm ){
  
  // Zero cmd
  if ( pwm == 0 ){
    digitalWrite(dri_pwm_pin, LOW);
    dri_standby = 1;
  }
  
  // Non-zero PWM
  else{
    
    // Wake-up PWM if if needed
    if (dri_standby == 1) {
      digitalWrite(dri_pwm_pin, HIGH);
      delayMicroseconds(dri_wakeup_time);
      dri_standby = 0;
    }
    
    // PWM direction
    if ( pwm < 0 ){
      digitalWrite(dri_dir_pin, HIGH);
    }
    else {
      digitalWrite(dri_dir_pin, LOW);
    }
    
    // Registery-based pwm duty cycle adjustement
    
    //Fast PWM, 9-bit, prescaler divider = 1
    TCCR4A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM41);
    TCCR4B = _BV(CS20) | _BV(WGM42);
  
    OCR4A = abs( pwm ) - 1; //set the duty cycle of pin 6
  }
  
}


///////////////////////////////////////////////////////////////////
// Read propulsion command from ROS
///////////////////////////////////////////////////////////////////
void cmdCallback ( const geometry_msgs::Twist&  twistMsg ){
  ser_ref  = twistMsg.angular.z; //rad
  dri_ref  = twistMsg.linear.x;  // volt or m/s or m
  ctl_mode = twistMsg.linear.z;  // 1    or 2   or 3
}

///////////////////////////////////////////////////////////////////
// Controller One tick
///////////////////////////////////////////////////////////////////
void ctl(){
  
  ///////////////////////////////////////////////
  // STEERING CONTROL
  /////////////////////////////////////////////// 
  
  // Servo Open-Loop fonction
  ser_pwm      = ser2pwm( ser_ref ) ;
  steeringServo.write(ser_pwm) ; 
  
  ///////////////////////////////////////////////
  // PROPULSION CONTROL
  /////////////////////////////////////////////// 
  
  // Retrieve current encoder counters
  enc_now = readEncoder();
  
  // Position computation
  pos_now = (float) enc_now * tick2m;
  
  // Velocity computation

  // TODO: COMPLETEZ LA DERIVEE FILTREE SUIVANTES
  float vel_raw = (enc_now - enc_old) * tick2m / time_period_low * 1000;
  float alpha   = 0;
  float vel_fil = vel_raw;    // Filtre a completer
  // FIN DU TODO
  
  // Propulsion Controllers
  
  //////////////////////////////////////////////////////
  if (ctl_mode == 0 ){
    // Zero output
    dri_pwm    = pwm_zer_dri ;
    
    // reset integral actions
    vel_error_int = 0;
    pos_error_int = 0 ;
    
  }
  //////////////////////////////////////////////////////
  else if (ctl_mode == 1 ){
    // Fully Open-Loop
    // Commands received in [Volts] directly
    dri_cmd    = dri_ref;
    dri_pwm    = cmd2pwm( dri_cmd ) ;
    
    // reset integral actions
    vel_error_int = 0;
    pos_error_int = 0 ;
  }
  //////////////////////////////////////////////////////
  else if (ctl_mode == 2 ){
    // Low-level Velocity control
    // Commands received in [m/sec] setpoints
    
    float vel_ref, vel_error;

    // TODO : ASSERVISSEMENT VITESSE
    vel_ref       = dri_ref; 
    vel_error     = 0;
    vel_error_int = 0;
    dri_cmd       = 0;
    // FIN DU TODO 
    
    dri_pwm    = cmd2pwm( dri_cmd ) ;

  }
  ///////////////////////////////////////////////////////
  else if (ctl_mode == 3){
    // Low-level Position control
    // Commands received in [m] setpoints
    
    float pos_ref, pos_error, pos_error_ddt;

    // TODO : ASSERVISSEMENT VITESSE
    pos_ref       = dri_ref; 
    // Anti wind-up??
    dri_cmd = 0;
    // FIN DU TODO
    
    dri_pwm = cmd2pwm( dri_cmd ) ;
  }
  ///////////////////////////////////////////////////////
  else if (ctl_mode == 4){
    // Reset encoder counts
    
    clearEncoderCount();
    
    // reset integral actions
    vel_error_int = 0 ;
    pos_error_int = 0 ;
    
    dri_pwm    = pwm_zer_dri ;
  }
  ////////////////////////////////////////////////////////
  else {
    // reset integral actions
    vel_error_int = 0 ;
    pos_error_int = 0 ;
    
    dri_pwm    = pwm_zer_dri ;
  }
  ///////////////////////////////////////////////////////
  
  // H-bridge pwm update
  set_pwm(dri_pwm);
  
  //Update memory variable
  enc_old = enc_now;

  // TODO UPDATE FILTER MEMORY
  vel_old = vel_raw;
  // FIN DU TODO
}


// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmdSubscriber("/prop_cmd", &cmdCallback) ;


///////////////////////////////////////////////////////////////////
// Arduino Initialization
///////////////////////////////////////////////////////////////////
void setup(){
  
  // Init PWM output Pins
  steeringServo.attach(ser_pin); 
  pinMode(dri_dir_pin, OUTPUT);
  pinMode(dri_pwm_pin, OUTPUT);
  
  // Init Communication
  Serial.begin(baud_rate);

  // Init and Clear Encoders
  initEncoder();    
  clearEncoderCount(); 
  
  // Init ROS
  nodeHandle.initNode();
  nodeHandle.subscribe(cmdSubscriber) ; // Subscribe to the steering and throttle messages
  nodeHandle.advertise(prop_sensors_pub);
  
  // Initialize Steering and drive cmd to neutral
  steeringServo.write(pwm_zer_ser) ;
  set_pwm(0);
  
  //
  delay(3000) ;
  
  nodeHandle.spinOnce();

}


////////////////////////////////////////////////////////////////////
//  Main Control Loop
////////////////////////////////////////////////////////////////////
void loop(){
  
  time_now = millis();

  ////////////////////////////////////////
  // Low-level controller
  ///////////////////////////////////////

  if (( time_now - time_last_low ) > time_period_low ) {
    
    ctl(); // one control tick

    time_last_low = time_now ;
  }

  ////////////////////////////////////////
  // Sync with ROS high-level controller
  ///////////////////////////////////////

  if (( time_now - time_last_high ) > time_period_high ) {

    // Feedback loop
    prop_sensors_data[0] = pos_now; // wheel position in m
    prop_sensors_data[1] = vel_old; // wheel velocity in m/sec
    
    // For DEBUG
    prop_sensors_data[2] = dri_ref; // set point received by arduino
    prop_sensors_data[3] = dri_cmd; // drive set point in volts
    prop_sensors_data[4] = dri_pwm; // drive set point in pwm
    prop_sensors_data[5] = enc_now; // raw encoder counts
    prop_sensors_data[6] = 0;
    prop_sensors_data[7] = 0;
    
    prop_sensors_msg.data        = &prop_sensors_data[0];
    prop_sensors_msg.data_length = prop_sensors_msg_length;
    prop_sensors_pub.publish( &prop_sensors_msg );
    
    // Read IMU ???
    
    //TODO
    
    // Publish IMU
    
    //TODO
    
    // Process ROS Events
    nodeHandle.spinOnce();

    time_last_high = time_now ;
  }
  
}
