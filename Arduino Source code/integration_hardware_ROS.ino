//ros header files
#define USE_USBCON

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>


//arduino header files
#include <SimpleKalmanFilter.h>
#include <Servo.h>
#include <EnableInterrupt.h>


//////////////////
/*Hardware:: Pin Setup*/
//////////////////
#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4


#define RELAY_SW_INPUT 11

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  0
#define RC_CH2_INPUT  1
#define RC_CH3_INPUT  2
#define RC_CH4_INPUT  3

#define esc_left_Pin 9
#define esc_right_Pin 10


//Kalmanfilter for joysyick data
SimpleKalmanFilter KalmanFilter_ch2(3, 3, 0.01);
SimpleKalmanFilter KalmanFilter_ch4(3, 3, 0.01);

//ESC Setup
Servo esc_left; 
Servo esc_right; 

//ESC::variables 
int minPulseRate = 768;
int maxPulseRate = 2400;

int safety_value =  1;

int stop_RC_Puls = 1060 + safety_value;
int full_RC_Puls = 1860 - safety_value;

int throttleChangeDelay = 100;

int left_initial_vel = 76;
int right_initial_vel = 76;


//Select Mode
bool relay_sw_trun_on_off = false;
bool audonomousMode_trun_on = false;
bool manualMode_trun_on = false;

//RC remote controller variables
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
uint16_t RC_1_data,RC_2_data,RC_3_data,RC_4_data;
uint16_t est_RC_1_data,est_RC_2_data,est_RC_3_data,est_RC_4_data;
int val_ch2, val_ch4;
int throttle_left, throttle_right;

//ROS variables
int16_t ros_pump_sw = 0;
uint16_t ros_throttle_left, ros_throttle_right =0;


///////////////////////////////////////////////
// ROS class and variable
///////////////////////////////////////////////

ros::NodeHandle nh;

std_msgs::UInt16MultiArray arr;
ros::Publisher pub_arr("/vL_vR", &arr);

std_msgs::Int16 usv_mode;
ros::Publisher pub_mode("/usv_mode", &usv_mode);

std_msgs::Int16 pump_sw;
ros::Publisher pub_pump_sw("/pump_sw", &pump_sw);


//Subscriber Part (event)
void messageController( const std_msgs::UInt16MultiArray& vel_array){
    ros_throttle_left = vel_array.data[0];
    ros_throttle_right = vel_array.data[1];
}
void messagePump( const std_msgs::Int16& vel_int){
    ros_pump_sw = vel_int.data;
}
ros::Subscriber <std_msgs::UInt16MultiArray> sub_velocity("/ros_vL_vR", &messageController);
ros::Subscriber <std_msgs::Int16> sub_pump("/ros_pump_sw", &messagePump);




//RC remote controller functions
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

//Transfrom joystick data into control data of both motors
void mapSpeeds() {
  throttle_left = 0;
  throttle_right = 0;
  int dir = (val_ch2>0) ? 1 : -1;
  if (abs(val_ch2) >= abs(val_ch4/2)) {
    throttle_left = val_ch2;
    throttle_right = val_ch2;
    if (val_ch4>0) {
      // decrease right wheels speed
      throttle_right -= (dir * val_ch4);
      throttle_right = (dir>0) ? max(throttle_right, 0) : min(throttle_right, 0);
      // increase left speed a bit if turning in an arc
      throttle_left += (dir * (val_ch4/2));
      throttle_left = (dir>0) ? min(throttle_left, 255) : max(throttle_left, -255);
    } else {
      // decrease left wheels speed
      throttle_left -= (dir * -val_ch4);
      throttle_left = (dir>0) ? max(throttle_left, 0) : min(throttle_left, 0);
      // increase right speed a bit if turning in an arc
      throttle_right += (dir * (-val_ch4/2));
      throttle_right = (dir>0) ? min(throttle_right, 255) : max(throttle_right, -255);
    }
  } 
  else 
  {
    throttle_left = val_ch4;
    throttle_right = -val_ch4;
  }
  throttle_right = normalizeThrottle(throttle_right); 
  throttle_left = normalizeThrottle(throttle_left); 
}

//to protect joystick values from excess of limitation
int normalizeThrottle(int value) {
  if( value < -90 )
    return -90;
  if( value > 90 )
    return 90;
  return value;
}

void setup() {
  
  //Serial.begin(SERIAL_PORT_SPEED);
  //Serial.setTimeout(50);
  
  esc_left.attach(esc_left_Pin,minPulseRate,maxPulseRate);
  esc_right.attach(esc_right_Pin,minPulseRate,maxPulseRate);
  
  esc_left.write(left_initial_vel);
  esc_right.write(right_initial_vel);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  pinMode(RELAY_SW_INPUT, OUTPUT);
  digitalWrite(RELAY_SW_INPUT,1);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);

  pinMode(13, OUTPUT);


///////////////////////////////////////////////
// ROS setup
///////////////////////////////////////////////

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  arr.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  arr.layout.dim[0].label = "USV_status";
  arr.layout.dim[0].size = 2;
  arr.layout.dim[0].stride = 1;
  arr.layout.data_offset = 0;
  arr.data = (int *)malloc(sizeof(int)*8);
  arr.data_length = 2;
 
  arr.data[0] = 0;   // left velocity of motor
  arr.data[1] = 0;  // right velocity of motor
  
  usv_mode.data = 0;
  pump_sw.data = 0; 

  //publish node
  nh.advertise(pub_arr); 
  nh.advertise(pub_mode); 
  nh.advertise(pub_pump_sw);

  //subscrive node
  nh.subscribe(sub_velocity);
  nh.subscribe(sub_pump);

}

void loop() {
  rc_read_values();

  //Read RC values
  RC_1_data = rc_values[RC_CH1];
  RC_2_data = rc_values[RC_CH2];
  RC_3_data = rc_values[RC_CH3];
  RC_4_data = rc_values[RC_CH4];

  //noise filtering from remote signals using Kalman filter
  //est_RC_1_data = KalmanFilter.updateEstimate(RC_1_data);
  est_RC_2_data = KalmanFilter_ch2.updateEstimate(RC_2_data);
  //est_RC_3_data = KalmanFilter.updateEstimate(RC_3_data);
  est_RC_4_data = KalmanFilter_ch4.updateEstimate(RC_4_data);

 
  // Turn on the water pump
  if(RC_1_data >= 1900 && RC_3_data <=1050 && audonomousMode_trun_on == false && manualMode_trun_on==true)
  {
    relay_sw_trun_on_off = true;
    digitalWrite(RELAY_SW_INPUT,0);
  }
  // Turn off the water pump
  if(RC_1_data <= 1100 && RC_3_data <=1050)
  {
    relay_sw_trun_on_off = false;
    digitalWrite(RELAY_SW_INPUT,1);
  }
  // Turn On manual mode
  if(RC_1_data >= 1900 && RC_3_data >=1900)
  {
    audonomousMode_trun_on = false;
    manualMode_trun_on = true;
    relay_sw_trun_on_off = false;
    digitalWrite(RELAY_SW_INPUT,1);
  }
  // Turn On autonomous mode
  if(RC_1_data <= 1100 && RC_3_data >=1900)
  {
    audonomousMode_trun_on = true;
    manualMode_trun_on = false;
    relay_sw_trun_on_off = false;
    digitalWrite(RELAY_SW_INPUT,1);
  }

  if(audonomousMode_trun_on == false && manualMode_trun_on == true)
  {
    ///////////////////////////////////////////////
    // display the sensor and controller data
    ///////////////////////////////////////////////
    
    /*
    //Serial.print("CH1:"); Serial.print(RC_1_data); Serial.print("\t");
    Serial.print("CH2:"); Serial.print(est_RC_2_data); Serial.print("\t");
    //Serial.print("CH3:"); Serial.print(RC_3_data); Serial.print("\t");
    Serial.print("CH4:"); Serial.print(est_RC_4_data); Serial.print("\t");
   
    if(relay_sw_trun_on_off == 1)
    {
      Serial.print("Pump:"); Serial.println("ON");
    }
    else if ((relay_sw_trun_on_off == 0))
    {
       Serial.print("Pump:"); Serial.println("OFF");
    }
    */
    
    val_ch2 = map(est_RC_2_data, 1000, 2000, -90, 90);
    val_ch4 = map(est_RC_4_data, 1000, 2000, -90, 90);
   
    mapSpeeds();
 
    left_initial_vel = map(throttle_left,-90, 90, stop_RC_Puls,full_RC_Puls);
    right_initial_vel = map(throttle_right,-90, 90, stop_RC_Puls,full_RC_Puls);
    esc_left.write(left_initial_vel);
    esc_right.write(right_initial_vel);
 
    usv_mode.data = 1;                  // 0 = Initial Mode, 1 = manual mode, 2 = ROS mode
    arr.data[0] = left_initial_vel;   // left velocity of motor
    arr.data[1] = right_initial_vel;  // right velocity of motor
    pump_sw.data = relay_sw_trun_on_off;                  // 0 = Turn on pump, 1 = Turn off pump
  }
  else if(audonomousMode_trun_on == true && manualMode_trun_on == false)
  {
    //Serial.println("ROS Mode");
    usv_mode.data = 2;                  // 0 = Initial Mode, 1 = manual mode, 2 = ROS mode

    //need to add topic scrib
    arr.data[0] = ros_throttle_left;   // left velocity of motor  (-90 ~ 0 ~ 90)
    arr.data[1] = ros_throttle_right;  // right velocity of motor (-90 ~ 0 ~ 90)
    pump_sw.data = ros_pump_sw;        // 0 = Turn on pump, 1 = Turn off pump
  
   
    left_initial_vel = map(ros_throttle_left,-90, 90, stop_RC_Puls,full_RC_Puls);
    right_initial_vel = map(ros_throttle_right,-90, 90, stop_RC_Puls,full_RC_Puls);
    esc_left.write(left_initial_vel);
    esc_right.write(right_initial_vel);
  
    digitalWrite(RELAY_SW_INPUT,ros_pump_sw); // 0 = Turn on pump, 1 = Turn off pump  
  }
  
  else
  {
    //Serial.println("Initial Mode");
    usv_mode.data = 0;                  // 0 = Initial Mode, 1 = manual mode, 2 = ROS mode
    arr.data[0] = 0;   // left velocity of motor
    arr.data[1] = 0;  // right velocity of motor
    pump_sw.data = 0;                  // 0 = Turn on pump, 1 = Turn off pump
  }

  pub_arr.publish(&arr);
  pub_mode.publish(&usv_mode);
  pub_pump_sw.publish(&pump_sw);

  nh.spinOnce();
  
  delay(50);
}
