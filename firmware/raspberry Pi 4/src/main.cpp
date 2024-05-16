#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>

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
#define MAX_MAPPED_VALUE 1760       // Minium value to which the joystick reading is mapped
#define MIN_MAPPED_VALUE 1200       // Maximum value to which the joystick reading is mapped
#define MAX_Z 1750
#define SERVO_OFF 1500 // 1500 // Value to write in order to stop the servo
#define DEPTH_SEND_INTERVAL 1000

// CONTROLLER VARIABLES
#define minForceZ -60
#define maxForceZ 74
#define minForceRoll -30
#define maxForceRoll 30
#define minForcePitch -30
#define maxForcePitch 30

// da regolare in base alla risposta dei sensori:
#define minErrorImu 0.03
#define minErrorBar 0.01
#define weight 171.10864
#define buoyancy 179.5909

#define MAX_SPEED 1.0
#define dt 0.03

#define sleepMillis(t) std::this_thread::sleep_for(std::chrono::milliseconds(t))

const std::string SERVER_ADDRESS	{ "tcp://10.0.0.254:1883" };
const std::string CLIENT_ID		    { "raspberry01" };
const std::string TOPIC_AXES 			{ "axes/" };  
const std::string TOPIC_COMMANDS 	{ "commands/" };  
const std::string TOPIC_PID 	    { "pid/" };  
const std::string TOPIC_CONFIG 	  { "config/" };  
const std::string TOPIC_DEBUG 	  { "debug/" };  
const std::string TOPIC_DEPTH 	  { "depth/" };

const int  QOS = 0;

// definition of the function to connect/reconnect to the mqtt server
void MQTT_connect();
void MQTT_reconnect();
long map_to(long x, long in_min, long in_max, long out_min, long out_max);
float normalize(float x, float in_min, float in_max, float out_min, float out_max);
float normalizeSqrt(long x);
float normalizeQuadratic(long x);
void loopMotori(msg_pt msg);
void loopBraccio(msg_pt msg);
void controlSystemCallFunction(ControlSystemZ zControl, ControlSystemPITCH pitchControl, ControlSystemROLL rollControl);
void readSensorsData();
int setBaselinePressure();
void changeControllerStatus(double depth, int Z_URemap);
int connectSerial();
void connectSerial1();
void my_handler(int s);
int motorSign (float v);

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

// Axes realated variable
int X, Y, Z_U, Z_D, ROLL, PITCH, YAW, WRIST;
int Z_URemap;
int Z_DRemap;

float XRemap, YRemap, YAWRemap;
float RDX_x, RDX_y, FDX_x, FDX_y, FSX_x, FSX_y, RSX_x, RSX_y, RDX, RSX, FDX, FSX;
int RDXsign, RSXsign, FDXsign, FSXsign;
float motor_power;

double depth, roll, pitch;
double referenceZ = 0;
double referencePitch = 0;
double referenceRoll = 0;
double last_depth_send_millis = 0;
float temperature_c;
float pressure_mbar;
float pressure_zero;
int res;
bool control_on = true;
bool globalControllerStatus = false;

char Debug[100];

//IMU
char const *dev = "/dev/i2c-1";
uint8_t addr = 0x50;

// json parsing related variables
int dim;
char *cmd;
json commandsIn; 
json pwdValues;
json armCommands;
json jsonConfig;

// Serial related variables
int fd;
const std::string serialPrefix = "/dev/ttyACM";
int serialConnected = 0; 

// Map string command to integer
std::map <std::string, int> mapper;

// mqtt and communication related variables
mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);
auto connOpts = mqtt::connect_options_builder()
	.clean_session(true)
	.finalize();


int main() {
  bool out;
  msg_pt msg;
 
  signal(SIGINT, my_handler);
  MQTT_connect();
  loadBaseConfig();
  connectSerial1();

  std::cout << (double)jsonConfig["denFHeave2"]<<std::endl << (double)jsonConfig["numFHeave1"]<<std::endl <<(double)jsonConfig["numFHeave2"]<<std::endl<<(double)jsonConfig["denCHeave2"]<<std::endl<<(double)jsonConfig["denCHeave3"]<<std::endl;
  
  // Init del controllore
  ControlSystemZ zControl = ControlSystemZ(minForceZ, maxForceZ, minErrorBar, weight, buoyancy, 
                                    jsonConfig["denFHeave2"], jsonConfig["numFHeave1"], jsonConfig["numFHeave2"], jsonConfig["denCHeave2"], jsonConfig["denCHeave3"], 
                                    jsonConfig["numCHeave2"], jsonConfig["numCHeave3"], jsonConfig["cZ_inf"]);                              
  ControlSystemPITCH pitchControl = ControlSystemPITCH(minForceRoll, maxForceRoll, minErrorImu, weight, buoyancy,
                                    jsonConfig["denFPitch2"], jsonConfig["denFPitch3"], jsonConfig["numFPitch2"], jsonConfig["numFPitch3"], jsonConfig["denCPitch2"], 
                                    jsonConfig["denCPitch3"], jsonConfig["numCPitch2"], jsonConfig["numCPitch3"], jsonConfig["cPITCH_inf"]);
  ControlSystemROLL rollControl = ControlSystemROLL(minForcePitch, maxForcePitch, minErrorImu, weight, buoyancy, 
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

  // Settare pressione iniziale per il calcolo della profondità
  pressure_zero = setBaselinePressure();

  mapper["ROTATE WRIST CCW"] = 0;
  mapper["ROTATE WRIST CW"] = 1;
  mapper["STOP WRIST"] = 2;
  mapper["OPEN NIPPER"] = 3;
  mapper["CLOSE NIPPER"] = 4;
  mapper["STOP NIPPER"] = 5;
  mapper["TORQUE_ON"] = 9;
  mapper["TORQUE_OFF"] = 7;
  mapper["None"] = 6;

  while (true) {
    MQTT_connect();

    if (cli.try_consume_message(&msg)) { 
      std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
      
      if (!msg) continue;    
      
      if (msg->get_topic() == TOPIC_COMMANDS)
        loopBraccio(msg);
      else if (msg->get_topic() == TOPIC_AXES) 
        loopMotori(msg);
      else if (msg->get_topic() == TOPIC_CONFIG)
        readConfig(msg);
    }
    
    // Leggi dati dai sensori e se attivo calcolare i pwm del controllore
    readSensorsData();
    if (jsonConfig["globalControllerStatus"]) 
      controlSystemCallFunction(zControl, pitchControl, rollControl);
    
    if(millis() - last_depth_send_millis > DEPTH_SEND_INTERVAL){
      last_depth_send_millis=millis();
      sprintf(Debug,"%.2f", depth);
      cli.publish(TOPIC_DEPTH, Debug);
      sprintf(Debug, "IMU roll: %.3f  pitch %.3f", roll, pitch);
      cli.publish(TOPIC_DEBUG, Debug);
    }

    sleepMillis(10);
  }
  return 0;
}


void loopBraccio(msg_pt msg) {
  armCommands["TYPE"] = 'B';
  armCommands["COMMAND"] = mapper[msg->to_string()];
  
  // Send via Serial JSON package
  std::string armString = armCommands.dump();
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
  Z_U = ((int)commandsIn["Z_UP"]);
  Z_D = ((int)commandsIn["Z_DOWN"]);
  Y = ((int)commandsIn["Y"]);
  X = ((int)commandsIn["X"]);
  YAW = ((int)commandsIn["YAW"]);

  // remap the recived values into a proper interval range, in order to process them
  //Z_URemap = map_to(Z_U, MIN_INPUT_READING, MAX_INPUT_READING, SERVO_OFF, MAX_Z);
  //Z_DRemap = map_to(Z_D, MIN_INPUT_READING, MAX_INPUT_READING, SERVO_OFF, MAX_Z);
  Z_URemap = normalizeSqrt(Z_U);
  Z_DRemap = normalizeSqrt(Z_D);
  if(Z_URemap==0) Z_URemap=1500;
  if(Z_DRemap==0) Z_DRemap=1500;

  // XRemap = normalize(X, MIN_INPUT_READING, MAX_INPUT_READING, -1, 1);
  // YRemap = normalize(Y, MIN_INPUT_READING, MAX_INPUT_READING, -1, 1);
  // YAWRemap = normalize(YAW, MIN_INPUT_READING, MAX_INPUT_READING, -1, 1);

  XRemap = normalizeQuadratic(X);
  YRemap = normalizeQuadratic(Y);
  YAWRemap = normalizeQuadratic(YAW);

  std::cout << "XRemap " << XRemap << std::endl;
  std::cout << "YRemap " << YRemap << std::endl;
  std::cout << "YAWRemap " << YAWRemap << std::endl;
  
  //calcolo componenti
  RDX_x = XRemap;
  RDX_y = YRemap*(-1);
  FDX_x = XRemap;
  FDX_y = YRemap;
  FSX_x = XRemap*(-1);
  FSX_y = YRemap;
  RSX_x = XRemap*(-1);
  RSX_y = YRemap*(-1);
  //calcolo versi dei motori
  RDXsign = motorSign(RDX_x + RDX_y);
  RSXsign = motorSign(RSX_x + RSX_y);
  FDXsign = motorSign(FDX_x + FDX_y);
  FSXsign = motorSign(FSX_x + FSX_y);

  //somma in quadratura
  motor_power = sqrt(((XRemap*XRemap) + (YRemap*YRemap)));
  RDX = RSX = FDX = FSX = motor_power;

  if(YAWRemap>0){
    if(RDXsign>=0)
      RDX += YAWRemap;
    else
      RDX -= YAWRemap;
    if(FSXsign>=0)
      FSX += YAWRemap;
    else
      FSX -= YAWRemap;
    if(FDXsign>=0)
      FDX += YAWRemap;
    else
      FDX -= YAWRemap;
    if(RSXsign>=0)
      RSX += YAWRemap;
    else
      RSX -= YAWRemap;
  }
  else{
    if(RDXsign>=0)
      RDX += YAWRemap;
    else
      RDX -= YAWRemap;
    if(FSXsign>=0)
      FSX += YAWRemap;
    else
      FSX -= YAWRemap;
    if(FDXsign>=0)
      FDX += YAWRemap;
    else
      FDX -= YAWRemap;
    if(RSXsign>=0)
      RSX += YAWRemap;
    else
      RSX -= YAWRemap;
  }

  RDX=RDX/(1+abs(YAWRemap));
  RSX=RSX/(1+abs(YAWRemap));
  FDX=FDX/(1+abs(YAWRemap));
  FSX=FSX/(1+abs(YAWRemap));

  std::cout << "RDX " << RDX << std::endl;
  std::cout << "RSX " << RSX << std::endl;
  std::cout << "FDX " << FDX << std::endl;
  std::cout << "FSX " << FSX << std::endl;

  if(RDXsign>=0)
    RDX = SERVO_OFF + RDX * (float)(MAX_MAPPED_VALUE-SERVO_OFF);
  else
    RDX = SERVO_OFF - RDX  * (float)(SERVO_OFF-MIN_MAPPED_VALUE);
  if(RSXsign>=0)
    RSX = SERVO_OFF + RSX * (float)(MAX_MAPPED_VALUE-SERVO_OFF);
  else
    RSX = SERVO_OFF - RSX * (float)(SERVO_OFF-MIN_MAPPED_VALUE);
  if(FDXsign>=0)
    FDX = SERVO_OFF + FDX * (float)(MAX_MAPPED_VALUE-SERVO_OFF);
  else
    FDX = SERVO_OFF - FDX * (float)(SERVO_OFF-MIN_MAPPED_VALUE);
  if(FSXsign>=0)
    FSX = SERVO_OFF + FSX * (float)(MAX_MAPPED_VALUE-SERVO_OFF);
  else
    FSX = SERVO_OFF - FSX * (float)(SERVO_OFF-MIN_MAPPED_VALUE);


  pwdValues["TYPE"] = 'A';
  pwdValues["FDX"] = 3000-(int)FDX;
  pwdValues["FSX"] = 3000-(int)FSX;
  pwdValues["RDX"] = 3000-(int)RDX;
  pwdValues["RSX"] = (int)RSX;

  // Z-Axis control
  if (Z_URemap >= Z_DRemap) {
    pwdValues["TYPE"] = 'A';
    pwdValues["UPFDX"] = Z_URemap;
    pwdValues["UPRSX"] = Z_URemap;
    pwdValues["UPRDX"] = Z_URemap;
    pwdValues["UPFSX"] = 3000 - Z_URemap;
  }
  else {
    pwdValues["TYPE"] = 'A';
    pwdValues["UPFDX"] = 3000 - Z_DRemap;
    pwdValues["UPRSX"] = 3000 - Z_DRemap;
    pwdValues["UPRDX"] = 3000 - Z_DRemap;
    pwdValues["UPFSX"] = Z_DRemap;
  }
  
  //changeControllerStatus(depth, Z_URemap);

  // Send via Serial JSON package
  std::string pwdString = pwdValues.dump();
  serialPuts(fd, pwdString.c_str());
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
  return (((float)(sqrt(x+MAX_INPUT_READING))/(float)sqrt(MAX_INPUT_READING*2))*(MAX_Z-SERVO_OFF))+SERVO_OFF;
}

void controlSystemCallFunction(ControlSystemZ zControl, ControlSystemPITCH pitchControl, ControlSystemROLL rollControl) {
  char DebugControllerInfo[200];
  
  if (control_on == true) {
    double forceZ = zControl.calculateZ(referenceZ, depth);
    double forcePitch = pitchControl.calculatePitch(referencePitch, pitch);
    double forceRoll = rollControl.calculateRoll(referenceRoll, roll);
    OutputValues pwm = compute_PWM(forceZ, forceRoll, forcePitch);
    
    pwdValues["UPFDX"] = (int) pwm.T5;
    pwdValues["UPRSX"] = (int) pwm.T8;
    pwdValues["UPRDX"] = (int) pwm.T7;
    pwdValues["UPFSX"] = (int) pwm.T6;
    
    sprintf(DebugControllerInfo,
            "{\"refZ\":%f, \"ForcePitch(N)\":%f,\"depth\":%f,\"roll\":%f,\"pitch\":%f,\"Z_Uremap\":%d,\"Z_DRemap\":%d}",
            referenceZ,
            forcePitch,
            depth,
            roll,
            pitch,
            Z_URemap,
            Z_DRemap);
    cli.publish(TOPIC_PID, DebugControllerInfo);
    
    std::string contrString = pwdValues.dump();
    serialPuts(fd, contrString.c_str());
    std::cout << "[CONTROLLER] " << contrString.c_str() << std::endl;
  }
}

void changeControllerStatus(double depth, int Z_URemap) {
  // Threshold for activating pid at certain depth
  if (depth < 0.3)
    control_on = false;
  else
    control_on = true;

  // Se il rov deve salire o scendere disattivare il controllore
  if (Z_URemap >= 1550 || Z_DRemap  >= 1550) {
    referenceZ = depth;
    control_on = false; 
  }
}

void readSensorsData(){
  WT61P_read_angle();
	double referenceRoll_new = WT61P_get_roll();
	double referencePitch_new = WT61P_get_pitch();

  //if no connection read are zeros, so exclude them and keep the previous
  if (referenceRoll_new != 0)
    roll = referenceRoll_new;
  if (referencePitch_new != 0)
    pitch = referencePitch_new * -1;
	//WT61P_get_yaw();

  /* read BAR data */
    res = ms5837_basic_read(&temperature_c, &pressure_mbar);
    if (res != 0) {
        //(void)ms5837_basic_deinit();
        //problemaaa
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
