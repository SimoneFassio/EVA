#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sstream>

#include "mqtt/async_client.h"
#include "json.hpp"
#include "CONTROLLER/Control_allocation.h"
#include "CONTROLLER/joystick_mapper.h"
#include "CONTROLLER/PolePlacementControl.h"
#include "CONTROLLER/reference_change.h"

#include "IMU/WIT61P.h"
#include "BAR/driver_ms5837_basic.h"


using json = nlohmann::json;
using msg_pt = std::shared_ptr<const mqtt::message>;

// Macro used to define contants used in the code
#define MIN_INPUT_READING -32678    // Minimum input reading value from the joystick
#define MAX_INPUT_READING 32678     // Maximum input reading value from the joystick
#define MAX_PWM 1730
#define MIN_PWM 1230
#define MAX_PWM_Z 1750
#define MIN_PWM_Z 1210
#define MAX_PWM_WORKMODE 1650
#define MIN_PWM_WORKMODE 1330
#define MAX_SLEW_RATE 100
#define SERVO_OFF 1500 // 1500 // Value to write in order to stop the servo
#define DEBUG_SEND_INTERVAL 100 //ms
#define INTERVAL_CONTROLLORE 10 //ms

// CONTROLLER VARIABLES
#define minForceZ -15
#define maxForceZ 15
#define minForceRoll -2
#define maxForceRoll 2
#define minForcePitch -5
#define maxForcePitch 5

#define DEGtoRAD 0.01745329f
#define PI 3.141592653589793f

// da regolare in base alla risposta dei sensori:
#define minErrorImu 0
#define minErrorBar 0
#define weight 171.10864
#define buoyancy 179.5909

#define MAX_SPEED 1.0
#define dt 0.03

#define sleepMillis(t) std::this_thread::sleep_for(std::chrono::milliseconds(t))

const std::string SERVER_ADDRESS	{ "tcp://10.0.0.254:1883" };
const std::string CLIENT_ID		    { "raspberry" };
const std::string TOPIC_AXES 			{ "axes/" };
const std::string TOPIC_COMMANDS 	{ "commands/" };
const std::string TOPIC_STATE_COMMANDS 	{ "state_commands/" };
const std::string TOPIC_PID 	    { "pid/" };
const std::string TOPIC_CONFIG 	  { "config/" };
const std::string TOPIC_DEBUG 	  { "debug/" };
const std::string TOPIC_GUI 	    { "gui/" };

const int  QOS = 0;

const double fixed_mixing_matrix[8][6] = {
    {cos(PI / 4), -sin(PI / 4), 0, 0, 0, 0.5}, //FSX
    {-cos(PI / 4), -sin(PI / 4), 0, 0, 0, -0.5},//FDX
    {-sin(PI / 4), -cos(PI / 4), 0, 0, 0, 0.5},//RSX
    {sin(PI / 4), -cos(PI / 4), 0, 0, 0, -0.5},//RDX
    {0, 0, 1, -1, 1, 0},//UPFSX
    {0, 0, 1, 1, 1, 0},//UPFDX
    {0, 0, 1, -1, -1, 0},//UPRSX
    {0, 0, 1, -1, 1, 0}//UPRDX
};
void normalize_vector(const double *input_array, double *output_array, uint8_t size)
{
    float max_abs_value = 0.0f;
    for (uint8_t i = 0; i < size; i++)
    {
        float abs_value = fabsf(input_array[i]);
        if (abs_value > max_abs_value)
        {
            max_abs_value = abs_value;
        }
    }

  if (max_abs_value > 1)
  {
    for (uint8_t i = 0; i < size; i++)
    {
      output_array[i] = input_array[i] / max_abs_value;
    }
  }
}

// definition of the function to connect/reconnect to the mqtt server
void MQTT_connect();
void MQTT_reconnect();
long map_to(long x, long in_min, long in_max, long out_min, long out_max);
float normalize(float x, float in_min, float in_max, float out_min, float out_max);
float normalizeSqrt(long x);
float normalizeQuadratic(long x);
void loopMotori(msg_pt msg);
void state_commands(msg_pt msg);
void loopBraccio(msg_pt msg);
void controlSystemCallFunction(ControlSystemZ zControl, ControlSystemPITCH pitchControl, ControlSystemROLL rollControl);
void readSensorsData();
int setBaselinePressure();
void changeControllerStatus(double depth, float Z_URemap);
int connectSerial();
void connectSerial1();
void my_handler(int s);
int motorSign (float v);
int reverseMotor(int pwm);
void checkSlewRate();

void readConfig(msg_pt configMsg);
void loadBaseConfig();


// motors position definition
// typedef enum {
//   FDX,
//   RSX,
//   RDX,
//   UPRSX,
//   FSX,
//   UPFDX,
//   UPFSX,
//   UPRDX
// } motors_position_mapping;

/*
Motors:

  - RDX -> Rear Right
  - FSX -> Forward Left
  - FDX -> Forward Right
  - UPRSX -> Up Rear Left
  - RSX -> Rear Left
  - UPFDX -> Up Rear Right
  - UPFSX -> Up Forward Left
  - UPRDX -> Up Rear Right

See the ROV picture for a proper understanding of the motors mapping
*/

  typedef enum {
  CHANGE_CONTROLLER_STATUS,
  ARM,
  DISARM,
  TRIM_PITCH_FORWARD,
  TRIM_PITCH_BACKWARD,
  TRIM_ROLL_LEFT,
  TRIM_ROLL_RIGHT,
  PWM_UP,
  PWM_DOWN,
  PWM_LEFT,
  PWM_RIGHT,
  WILDCARD
} state_commands_map;

const char motors[][6]= {"FSX", "FDX", "RSX", "RDX", "UPFDX", "UPFSX", "UPRDX", "UPRSX"};

int armed=0;  //if =1 rov attivo
int previousArm=0;
int trimPitch=0;
int trimRoll=0;

// Axes realated variable
int X, Y, Z, YAW;

float XRemap, YRemap, YAWRemap, ZRemap;
float RDX_x, RDX_y, FDX_x, FDX_y, FSX_x, FSX_y, RSX_x, RSX_y, RDX, RSX, FDX, FSX;
int RDXsign, RSXsign, FDXsign, FSXsign;
float motor_power;
double last_controller_time=0;

double depth, roll, pitch, yaw;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double temperature_IMU;
double referenceZ = 0;
double referencePitch = 0;
double referenceRoll = 0;
double last_debug_send_millis = 0;
float temperature_c;
float pressure_mbar;
float pressure_zero;
int res;
bool control_on = false;

int max_pwm = MAX_PWM;
int min_pwm = MIN_PWM;

char Debug[200];

//IMU
char const *dev = "/dev/i2c-1";
uint8_t addr = 0x50;

// json parsing related variables
int dim;
char *cmd;
json commandsIn; 
json pwdValues;
json pwdValuesLastSend;
json armCommands;
json jsonConfig;

// Serial related variables
int fd;
const std::string serialPrefix = "/dev/ttyACM";
int serialConnected = 0; 

// Map string command to integer
std::map <std::string, int> mapper;
std::map <std::string, state_commands_map> state_mapper;

// mqtt and communication related variables
mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);
auto connOpts = mqtt::connect_options_builder()
	.clean_session(true)
	.finalize();


int main() {
  bool out;
  double milliss;
  msg_pt msg;
  
  signal(SIGINT, my_handler);
  MQTT_connect();
  loadBaseConfig();
  connectSerial1();

  jsonConfig["globalControllerStatus"];

  
  // Init del controllore
  ControlSystemZ zControl = ControlSystemZ(minForceZ, maxForceZ, minErrorBar, weight, buoyancy, 
                                    jsonConfig["denFHeave2"], jsonConfig["numFHeave1"], jsonConfig["numFHeave2"], jsonConfig["denCHeave2"], jsonConfig["denCHeave3"], 
                                    jsonConfig["numCHeave2"], jsonConfig["numCHeave3"], jsonConfig["cZ_inf"]);                              
  ControlSystemPITCH pitchControl = ControlSystemPITCH(minForcePitch, maxForcePitch, minErrorImu*DEGtoRAD, weight, buoyancy,
                                    jsonConfig["denFPitch2"], jsonConfig["denFPitch3"], jsonConfig["numFPitch2"], jsonConfig["numFPitch3"], jsonConfig["denCPitch2"], 
                                    jsonConfig["denCPitch3"], jsonConfig["numCPitch2"], jsonConfig["numCPitch3"], jsonConfig["cPITCH_inf"]);
  ControlSystemROLL rollControl = ControlSystemROLL(minForceRoll, maxForceRoll, minErrorImu*DEGtoRAD, weight, buoyancy, 
                                    jsonConfig["denCRoll2"], jsonConfig["denCRoll3"], jsonConfig["numCRoll2"], jsonConfig["numCRoll3"], jsonConfig["cROLL_inf"]);
  
  res = WT61P_begin(const_cast<char*>(dev), addr);
  if (res != 0) {
    jsonConfig["globalControllerStatus"] = false; // Se fallisce init barometro disattivare controllore 
    sprintf(Debug,"NO IMU");
    cli.publish(TOPIC_DEBUG, Debug);
  }
  else {
    sprintf(Debug,"IMU OK");
    cli.publish(TOPIC_DEBUG, Debug);
  }

  res = ms5837_basic_init(MS5837_TYPE_02BA01);
  if (res != 0) {
    jsonConfig["globalControllerStatus"] = false; // Se fallisce init barometro disattivare controllore 
    sprintf(Debug,"NO BAROMETRO");
    cli.publish(TOPIC_DEBUG, Debug);
  }
  else {
    sprintf(Debug,"BAROMETRO OK");
    cli.publish(TOPIC_DEBUG, Debug);
  }

  mapper["None"] = 6;
  mapper["ROTATE_WRIST_CCW"] = 1;
  mapper["ROTATE_WRIST_CW"] = 0;
  mapper["STOP_WRIST"] = 2;
  mapper["OPEN_NIPPER"] = 4;
  mapper["CLOSE_NIPPER"] = 3;
  mapper["STOP_NIPPER"] = 5;
  mapper["TORQUE_WRIST_ON"] = 9;
  mapper["TORQUE_WRIST_OFF"] = 7;


  state_mapper["CHANGE_CONTROLLER_STATUS"] = CHANGE_CONTROLLER_STATUS;
  state_mapper["ARM"] = ARM;
  state_mapper["DISARM"] = DISARM;
  state_mapper["TRIM_PITCH_FORWARD"] = TRIM_PITCH_FORWARD;
  state_mapper["TRIM_PITCH_BACKWARD"] =TRIM_PITCH_BACKWARD;
  state_mapper["TRIM_ROLL_LEFT"] = TRIM_ROLL_LEFT;
  state_mapper["TRIM_ROLL_RIGHT"] = TRIM_ROLL_RIGHT;
  state_mapper["PWM_UP"] = PWM_UP;
  state_mapper["PWM_DOWN"] = PWM_DOWN;
  state_mapper["PWM_RIGHT"] = PWM_RIGHT;
  state_mapper["PWM_LEFT"] = PWM_LEFT;
  state_mapper["SNAP"] = WILDCARD;


  pwdValuesLastSend["FDX"] = SERVO_OFF;
  pwdValuesLastSend["FSX"] = SERVO_OFF;
  pwdValuesLastSend["RDX"] = SERVO_OFF;
  pwdValuesLastSend["RSX"] = SERVO_OFF;
  pwdValuesLastSend["UPFDX"] = SERVO_OFF;
  pwdValuesLastSend["UPRSX"] = SERVO_OFF;
  pwdValuesLastSend["UPRDX"] = SERVO_OFF;
  pwdValuesLastSend["UPFSX"] = SERVO_OFF;

  

  while (true) {
    
    milliss = millis();
    MQTT_connect();

    if (cli.try_consume_message(&msg)) { 
      std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
      
      if (!msg) continue;   

      if (msg->get_topic() == TOPIC_STATE_COMMANDS) 
          state_commands(msg); 
      else if (msg->get_topic() == TOPIC_CONFIG)
          readConfig(msg);

      if(armed){
        if (msg->get_topic() == "commands/") {
          loopBraccio(msg); }
        else if (msg->get_topic() == TOPIC_AXES) 
          loopMotori(msg);

        if(previousArm == 0 && armed == 1){ // Settare pressione iniziale per il calcolo della profondità
          pressure_zero = setBaselinePressure();
          previousArm=1;
        }
      }
    }
    
    // Leggi dati dai sensori e se attivo calcolare i pwm del controllore
    if(milliss - last_controller_time > INTERVAL_CONTROLLORE){
        last_controller_time = milliss;
        readSensorsData();
        if (armed) {
          if (jsonConfig["globalControllerStatus"])
            controlSystemCallFunction(zControl, pitchControl, rollControl);
          //send PWM serial
          //checkSlewRate();
          std::string pwdString = pwdValues.dump();
          serialPuts(fd, pwdString.c_str());
      }
      if(milliss - last_debug_send_millis > DEBUG_SEND_INTERVAL){
        last_debug_send_millis=milliss;
        int pidState=0;
        if(jsonConfig["globalControllerStatus"]){
          pidState=1;
          if(control_on)
            pidState=2;
        }
        sprintf(Debug,
          "{\"pidState\":%d, \"armed\":%d, \"depth\":%.2f, \"yaw\":%.2f, \"roll\":%.2f, \"pitch\":%.2f,\"tempInt\":%.2f,\"tempExt\":%.2f}",
          pidState,
          armed,
          depth,
          yaw,
          roll/DEGtoRAD,
          pitch/DEGtoRAD,
          temperature_IMU,
          temperature_c);
        cli.publish(TOPIC_GUI, Debug);
        /*sprintf(Debug,
            "{\"refZ\":%f, \"refRoll\":%f, \"refPitch\":%f, \"ForceZ(N)\":%f, \"ForceRoll(N)\":%f, \"ForcePitch(N)\":%f,\"depth\":%f,\"roll\":%f,\"pitch\":%f,\"ZRemap\":%d}",
            0,
            referenceRoll,
            referencePitch,
            0,
            0,
            0,
            accZ-1,
            accY,
            accX,
            ZRemap);
    cli.publish(TOPIC_PID, Debug);*/
      }
    }
  }
  return 0;
}


void loopBraccio(msg_pt msg) {
  armCommands["TYPE"] = 'B';
  armCommands["COMMAND"] = mapper[msg->to_string()];
  
  // Send via Serial JSON package
  std::string armString = armCommands.dump();
  std::cout << armString << std::endl;
  serialPuts(fd, armString.c_str());
  std::cout << "[ARM] " << armString.c_str() << std::endl;
}

void loopMotori(msg_pt msg) {

  commandsIn = json::parse(msg->to_string(), nullptr, false); 
  if(commandsIn.is_discarded()){
    std::cout << "parse error" << std::endl;
    return;
  }
  // parse the values recived into the allocated variable
  Z = ((int)commandsIn["Z"]);
  Y = ((int)commandsIn["Y"]);
  X = ((int)commandsIn["X"]);
  YAW = ((int)commandsIn["YAW"]);

  // remap the received values into a proper interval range, in order to process them

  // XRemap = normalize(X, MIN_INPUT_READING, MAX_INPUT_READING, -1, 1);
  // YRemap = normalize(Y, MIN_INPUT_READING, MAX_INPUT_READING, -1, 1);
  // YAWRemap = normalize(YAW, MIN_INPUT_READING, MAX_INPUT_READING, -1, 1);

  XRemap = normalizeQuadratic(X);
  YRemap = normalizeQuadratic(Y);
  YAWRemap = normalizeQuadratic(YAW);
  ZRemap = normalize(Z, MIN_INPUT_READING, MAX_INPUT_READING, 1, -1);

  double joystick_input[6] = {YRemap, XRemap, ZRemap, 0, 0, YAWRemap};

  // Define the result array to store the result of multiplication
  double result[8] = {0};

  // Perform matrix multiplication
  for (int i = 0; i < 8; ++i) {
      result[i] = 0;  // Initialize the result for the current row
      for (int j = 0; j < 6; ++j) {
          result[i] += fixed_mixing_matrix[i][j] * joystick_input[j];
      }
  }

  normalize_vector(result, result, 8);

  for(int i=0; i< 8; i++)
  {
    std::cout << i << ": " << result[i] << std::endl;
  }

  FSX = normalize(result[0], -1.0, 1.0, min_pwm, max_pwm);
  FDX = normalize(result[1], -1.0, 1.0, min_pwm, max_pwm);
  RSX = normalize(result[2], -1.0, 1.0, min_pwm, max_pwm);
  RDX = normalize(result[3], -1.0, 1.0, min_pwm, max_pwm);



  pwdValues["TYPE"] = 'A';
  pwdValues["FDX"] = (int)FDX;
  pwdValues["FSX"] = reverseMotor(FSX);
  pwdValues["RDX"] = (int)RDX;
  pwdValues["RSX"] = (int)RSX;

  // Z-Axis control
  double UPFSX = normalize(result[4], -1.0, 1.0, min_pwm, max_pwm);
  double UPFDX = normalize(result[5], -1.0, 1.0, min_pwm, max_pwm);
  double UPRSX = normalize(result[6], -1.0, 1.0, min_pwm, max_pwm);
  double UPRDX = normalize(result[7], -1.0, 1.0, min_pwm, max_pwm);
  
  changeControllerStatus(depth, ZRemap);

  pwdValues["TYPE"] = 'A';
  pwdValues["UPFDX"] = (int)UPFDX;
  pwdValues["UPRSX"] = (int)UPRSX;
  pwdValues["UPRDX"] = (int)UPRDX;
  pwdValues["UPFSX"] = reverseMotor(UPFSX);

  std::string pwdString = pwdValues.dump();
  std::cout << "[MOTORS] " << pwdString.c_str() << std::endl;
}

// the following function is used in order to connect to mqtt server or reconnect to it
// in case the connection is lost:
// hence the function is called at the beginning of each iteration of the loop function
void MQTT_connect()
{
  if (cli.is_connected()) return;
	
  else {
    try {
      // Start consumer before connecting to make sure to not miss messages
      cli.start_consuming();

      // Connect to the server
      std::cout << "[MQTT] " << "Connecting to the MQTT server..." << std::endl;
      auto tok = cli.connect(connOpts);

      // Getting the connect response will block waiting for the connection to complete.
      auto rsp = tok->get_connect_response();

      // If there is no session present, then we need to subscribe, but if
      // there is a session, then the server remembers us and our subscriptions.
      if (!rsp.is_session_present())
        cli.subscribe(TOPIC_AXES, QOS)->wait();
        cli.subscribe(TOPIC_COMMANDS, QOS)->wait();
        cli.subscribe(TOPIC_STATE_COMMANDS, QOS)->wait();

      std::cout << "[MQTT] " << "Connection established" << std::endl;

    }
    catch (const mqtt::exception& exc) {
      std::cerr << "\n  " << exc << std::endl;
      exit(-1);
    }
	}
}

void MQTT_reconnect(){
    // If we're here, the client was almost certainly disconnected.
		// But we check, just to make sure.

		if (cli.is_connected()) {
			std::cout << "\nShutting down and disconnecting from the MQTT server..." << std::endl;
			cli.unsubscribe(TOPIC_AXES)->wait();
			cli.unsubscribe(TOPIC_COMMANDS)->wait();
			cli.stop_consuming();
			cli.disconnect()->wait();
			std::cout << "OK" << std::endl;
		}
		else {
			std::cout << "\nClient was disconnected" << std::endl;
      MQTT_connect();
		}
}

long map_to(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float normalize(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float normalizeQuadratic(long x) {
  int sign=1;
  if(x>0)
    sign = 1;
  else
    sign = -1;
  
  return (float)(x*x)/(float)(MAX_INPUT_READING*MAX_INPUT_READING)*sign;
}

float normalizeSqrt(long x) {
  return (((float)(sqrt(x+MAX_INPUT_READING))/(float)sqrt(MAX_INPUT_READING*2))*(MAX_PWM_Z-SERVO_OFF))+SERVO_OFF;
}

void controlSystemCallFunction(ControlSystemZ zControl, ControlSystemPITCH pitchControl, ControlSystemROLL rollControl) {
  char DebugControllerInfo[200];
  
  if (control_on == true) {
    double forceZ = zControl.calculateZ(referenceZ*10, depth);
    double forcePitch = 0; //pitchControl.calculatePitch(referencePitch, pitch);
    double forceRoll = 0; // rollControl.calculateRoll(referenceRoll, roll);
    OutputValues pwm = compute_PWM(forceZ, forceRoll, forcePitch);
    
    printf("[PWM PID] UPFDX: %d | UPRSX: %.2f | UPRDX: %.2f | UPFSX: %.2f\n", (int) pwm.T5, pwm.T6, pwm.T7, pwm.T8);

    pwdValues["UPFDX"] = reverseMotor(pwm.T5);
    pwdValues["UPRSX"] = reverseMotor(pwm.T8);
    pwdValues["UPRDX"] = reverseMotor(pwm.T7);
    pwdValues["UPFSX"] = (int) pwm.T6;
    
    sprintf(DebugControllerInfo,
             "{\"refZ\":%f, \"refRoll\":%f, \"refPitch\":%f, \"ForceZ(N)\":%f, \"ForceRoll(N)\":%f, \"ForcePitch(N)\":%f,\"depth\":%f,\"roll\":%f,\"pitch\":%f,\"ZRemap\":%d,\"accX\":%d,\"accY\":%d,\"accZ\":%d}",
            referenceZ,
            referenceRoll,
            referencePitch,
            forceZ,
            forceRoll,
            forcePitch,
            depth,
            roll/DEGtoRAD,
            pitch/DEGtoRAD,
            ZRemap,
            accX, accY, accZ);
    
    
    std::string contrString = pwdValues.dump();
    cli.publish(TOPIC_PID, DebugControllerInfo);
    std::cout << "[CONTROLLER] " << contrString.c_str() << std::endl;
  }
  
}

void changeControllerStatus(double depth, float ZRemap) {
  //std::cout << "zremap " << ZRemap << std::endl;
  // Threshold for activating pid at certain depth
  if (abs(ZRemap) < 0.02  && depth > 0.1) {
    if (control_on==false)
      referenceZ = depth;
    control_on = true; 
  }
  else
    control_on = false;
}

void readSensorsData(){
  //WT61P_read_angle();
  WT61P_read_all();
	double roll_new = WT61P_get_roll();
	double pitch_new = WT61P_get_pitch();
  double yaw_new = WT61P_get_yaw();

  //if no connection read are zeros, so exclude them and keep the previous
  if (roll_new != 0)
    pitch = roll_new * DEGtoRAD;  //swap pitch roll
  if (yaw_new != 0)
    yaw = yaw_new;
  if (pitch_new != 0)
    roll = (pitch_new) * DEGtoRAD;

  accX = WT61P_get_AX();
  accY = WT61P_get_AY();
  accZ = WT61P_get_AZ();

  gyroX = WT61P_get_GX();
  gyroY = WT61P_get_GY();
  gyroZ = WT61P_get_GZ();

  temperature_IMU = WT61P_get_temp();

  /* read BAR data */
    res = ms5837_basic_read(&temperature_c, &pressure_mbar);
    if (res != 0) {
        //(void)ms5837_basic_deinit();
        //problemaaa
        return;
    }
    else
      depth = (pressure_mbar-pressure_zero)*100/(9.80665*997.0f);  //997=density fresh water
    
}

int setBaselinePressure() {
  int times = 15; int sum = 0;
  float pressureTmp;

  // Prenderla più volte per avere una misurazione più precisa, poi farne la media
  for (int i = 0; i < times; i++) {
    ms5837_basic_read(&temperature_c, &pressureTmp);
    sum += pressureTmp;
    //std::cout << "read" << pressureTmp << " :: " << temperature_c << std::endl; 
  }
  
  return (float) sum / times;
}

int connectSerial() {
  std::string serialInteface;
  int maxTries = 4;

  std::cout << "[SERIAL] Trying connecting to serial..." << std::endl; 
  //serialInteface = serialPrefix + std::to_string(0);
  //std::cout << "[SERIAL] " << serialInteface.c_str() << std::endl;
  //fd = serialOpen(serialInteface.c_str(), 115200);
  //fd = serialOpen("/dev/ttyACM0", 115200);
  
  // Try to connect to differet serial interfaces
  for (int i = 0; i < maxTries && !serialConnected; i++) {
    serialInteface = serialPrefix + std::to_string(i);
    if (fd = serialOpen(serialInteface.c_str(), 115200) > 0) {
      std::cout << "[SERIAL] " << "Connected to:" << serialInteface << std::endl;
      serialConnected = 1;
    }
    sleepMillis(20);
  }
  
  std::cout << "[SERIAL] " << serialConnected << std::endl;
  return serialConnected;   
}

void connectSerial1() {
  fd = serialOpen("/dev/ttyACM0", 115200);
  if(fd>=0){
    std::cout << "serial /dev/ttyACM0 open fd: " << fd <<  std::endl;
    sprintf(Debug,"serial /dev/ttyACM0 open");
    cli.publish(TOPIC_DEBUG, Debug);
  }
  else{
    sleepMillis(20);
    fd = serialOpen("/dev/ttyACM1", 115200);
    if(fd>=0){
      std::cout << "serial /dev/ttyACM1 open" << std::endl;
      sprintf(Debug,"serial /dev/ttyACM1 open");
      cli.publish(TOPIC_DEBUG, Debug);
    }
    else
      std::cout << "NO SERIAL CONNECTION" << std::endl;
      sprintf(Debug,"NO SERIAL CONNECTION");
      cli.publish(TOPIC_DEBUG, Debug);
  }
}

void loadBaseConfig() {
  std::ifstream jsonConfigFile("/home/politocean/firmware/config/baseControllerConfig.json");
  
  if (jsonConfigFile.is_open()) {
    std::cout << "[CONFIG] Reading config file" << std::endl;
    jsonConfigFile >> jsonConfig;
    jsonConfigFile.close();
  }
  std::cout << jsonConfig.dump().c_str() << std::endl; 
}

void readConfig(msg_pt msg) {
  jsonConfig = json::parse(msg->to_string());
}

// Handler in caso di ctrl-C per chiusura corretta della connessione seriale
void my_handler(int s){
  serialClose(fd);
  exit(1); 
}

int motorSign (float v){
  if (v == 0)
    return 0;
  if (v > 0)
    return 1;
  return -1;
}

void state_commands(msg_pt msg){
  state_commands_map cmd = state_mapper[msg->to_string()];
  switch(cmd){
    case ARM:
      previousArm = armed;
      armed = 1;
      sprintf(Debug,"ROV_ARMED");
      cli.publish(TOPIC_DEBUG, Debug);
      break;
    case DISARM:
      previousArm = armed;
      armed = 0;
      sprintf(Debug,"ROV_DISARMED");
      cli.publish(TOPIC_DEBUG, Debug);
      break;
    case CHANGE_CONTROLLER_STATUS:
      jsonConfig["globalControllerStatus"] = !jsonConfig["globalControllerStatus"];
      if(jsonConfig["globalControllerStatus"]){
        max_pwm = MAX_PWM_WORKMODE;
        min_pwm = MIN_PWM_WORKMODE;
      }
      else{
        max_pwm=MAX_PWM;
        min_pwm=MIN_PWM;
      }
      sprintf(Debug,"controller status: %d", (int)jsonConfig["globalControllerStatus"]);
      cli.publish(TOPIC_DEBUG, Debug);
      break;
    case TRIM_PITCH_FORWARD: //dpad up
      referenceZ = referenceZ - 0.02;
      // trimPitch+=5;
      // sprintf(Debug,"trimPitch: %d", trimPitch);
      // cli.publish(TOPIC_DEBUG, Debug);
      break;
    case TRIM_PITCH_BACKWARD: //dpad down
      referenceZ = referenceZ + 0.02;
      // trimPitch-=5;
      // sprintf(Debug,"trimPitch: %d", trimPitch);
      // cli.publish(TOPIC_DEBUG, Debug);
      break;
    case TRIM_ROLL_LEFT:
      trimRoll+=5;
      sprintf(Debug,"trimRoll: %d", trimRoll);
      cli.publish(TOPIC_DEBUG, Debug);
      break;
    case TRIM_ROLL_RIGHT:
      trimRoll-=5;
      sprintf(Debug,"trimRoll: %d", trimRoll);
      cli.publish(TOPIC_DEBUG, Debug);
      break;
    case PWM_UP:
      max_pwm+=50;
      min_pwm-=50;

      if(max_pwm>MAX_PWM)
        max_pwm=MAX_PWM;
      if(min_pwm<MIN_PWM)
        min_pwm=MIN_PWM;
      break;
    case PWM_DOWN:
      max_pwm-=50;
      min_pwm+=50;

      if(max_pwm<SERVO_OFF+50)
        max_pwm=SERVO_OFF+50;
      if(min_pwm>SERVO_OFF-50)
        min_pwm=SERVO_OFF-50;
      break;
    case PWM_LEFT:
      max_pwm=MAX_PWM;
      min_pwm=MIN_PWM;
      break;
    case PWM_RIGHT:
      max_pwm=MAX_PWM-120;
      min_pwm=MIN_PWM+120;
      break;
    default:
      break;
  }
}

int reverseMotor(int pwm){
  if(pwm>SERVO_OFF){
    float m = (float)(pwm-SERVO_OFF)/(float)(max_pwm-SERVO_OFF);
    return SERVO_OFF - m*(SERVO_OFF-min_pwm);
  }
  else{
    float m = (float)(SERVO_OFF-pwm)/(float)(SERVO_OFF-min_pwm);
    return SERVO_OFF + m*(max_pwm-SERVO_OFF);
  }
}

void checkSlewRate(){ //does not work
  std::stringstream ss1("");
  std::stringstream ss("");
  for(int i=0; i<8; i++){
    ss << pwdValuesLastSend[motors[i]];
    int pwdls = 0;
    ss >> pwdls;
    ss1 << pwdValues[motors[i]];
    int pwd = 0;
    ss1 >> pwd;
    //std::cout << "pwd " << pwd << "pwdls " << pwdls << std::endl;
    if(pwdls>SERVO_OFF){
      if(pwd>(pwdls+MAX_SLEW_RATE))
        pwdValues[motors[i]] = (int)pwdls + MAX_SLEW_RATE;
    }
    else{
      if(pwd<(pwdls-MAX_SLEW_RATE))
        pwdValues[motors[i]] = (int)pwdls-MAX_SLEW_RATE;
    }
    pwdValuesLastSend[motors[i]] = pwd;
  }
}
