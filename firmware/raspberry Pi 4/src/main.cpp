#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#include "mqtt/async_client.h"
#include "json.hpp"
#include "control_allocation.h"
#include "joystick_mapper.h"
#include "PolePlacementControl.h"
#include "reference_change.h"

#include "IMU/WIT61P.h"
#include "BAR/driver_ms5837_basic.h"


using json = nlohmann::json;
using msg_pt = std::shared_ptr<const mqtt::message>;

// Macro used to define contants used in the code
#define NUM_SERVO 8
#define MQTT_TIMEOUT 10             // milliseconds
#define MQTT_CONNECT_RETRY_DELAY 15 // milliseconds
#define ESC_DELAY 7000              // millisecons
#define MIN_INPUT_READING -32678    // Minimum input reading value from the joystick
#define MAX_INPUT_READING 32678     // Maximum input reading value from the joystick
#define MIN_MAPPED_VALUE 1760       // Minium value to which the joystick reading is mapped
#define MAX_MAPPED_VALUE 1200       // Maximum value to which the joystick reading is mapped
#define MAX_Z 1750
#define SERVO_OFF 1500 // 1500 // Value to write in order to stop the servo

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
#define weight 162.846
#define buoyancy 168.9282

#define MAX_SPEED 1.0
#define dt 0.03

#define map_pwm(X) X == 1500 ? 0.0 : ((X / (1900.0 + 1100.0)) * dt * MAX_SPEED * (X >= 1500 ? 1.0 : -1.0))
#define sleepMillis(t) std::this_thread::sleep_for(std::chrono::milliseconds(t))

const std::string SERVER_ADDRESS	  { "tcp://10.0.0.254:1883" };
const std::string CLIENT_ID		    { "raspberry01" };
const std::string TOPIC_AXES 			{ "axes/" };  
const std::string TOPIC_COMMANDS 	{ "commands/" };  
const std::string TOPIC_PID 	      { "pid/" };  
const std::string TOPIC_CONFIG 	  { "pid/" };  

const int  QOS = 0;

// definition of the function to connect/reconnect to the mqtt server
void MQTT_connect();
void MQTT_reconnect();
long map_to(long x, long in_min, long in_max, long out_min, long out_max);
void loopMotori(msg_pt msg);
void loopBraccio(msg_pt msg);
void controlSystemCallFunction();
void readSensorsData();
int setBaselinePressure();
void changeControllerStatus(double depth, int Z_URemap);
int connectSerial();
void connectSerial1();
void my_handler(int s);

// motors position definition
typedef enum {
  FDX,
  RSX,
  RDX,
  UPRSX,
  FSX,
  UPFDX,
  UPFSX,
  UPRDX
} motors_position_mapping;

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
int YRemap;
int XRemap;
float YRemap2;
float XRemap2;

float xp, yp;
float valx1, valy1;
float valx2, valy2;

double depth, roll, pitch;
double referenceZ = 0;
double referencePitch = 0;
double referenceRoll = 0;
float temperature_c;
float pressure_mbar;
float pressure_zero;
int res;
bool control_on = true;
bool globalControllerStatus = false;


//IMU
char const *dev = "/dev/i2c-1";
uint8_t addr = 0x50;

// json parsing related variables
int dim;
char *cmd;
json commandsIn; 
json pwdValues;
json armCommands;

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

ControlSystem zControl = ControlSystem(minForceZ, maxForceZ, minErrorBar, weight, buoyancy);
ControlSystem rollControl = ControlSystem(minForceRoll, maxForceRoll, minErrorImu, weight, buoyancy);
ControlSystem pitchControl = ControlSystem(minForcePitch, maxForcePitch, minErrorImu, weight, buoyancy);

int main() {

  signal (SIGINT,my_handler);

  bool out;
  msg_pt msg;
  //connectSerial();
  //serialClose(fd);
  //std::cout << serialDataAvail(fd) << std::endl;
  //fd = serialOpen("/dev/ttyACM0", 115200);
  connectSerial1();
  WT61P_begin(const_cast<char*>(dev), addr);
  res = ms5837_basic_init(MS5837_TYPE_30BA26);

  if (res != 0) 
    globalControllerStatus = false; // Se fallisce init barometro disattivare controllore 

  // Settare pressione iniziale per il calcolo della profondità
  pressure_zero = setBaselinePressure();

  //!!!! Add Wildcard for null command, using !!!
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
    // Connect MQTT
    MQTT_connect();

    //out = cli.try_consume_message(&msg);
    if (cli.try_consume_message(&msg)) { 
      std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
      
      if (!msg) {
        MQTT_reconnect();
        std::cout << "!msg" << std::endl;
        continue;    
      }
      
      if (msg->get_topic() == TOPIC_COMMANDS)
        loopBraccio(msg);
      else if (msg->get_topic() == TOPIC_AXES) 
        loopMotori(msg);
    }
    
    // Leggi dati dai sensori e se attivo calcolare i pwm del controllore
    readSensorsData();
    if (globalControllerStatus) 
      controlSystemCallFunction();
    
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

  // remap the recived values into a proper interval range, in order to process them
  Z_URemap = map_to(Z_U, MIN_INPUT_READING, MAX_INPUT_READING, SERVO_OFF, MAX_Z);
  Z_DRemap = map_to(Z_D, MIN_INPUT_READING, MAX_INPUT_READING, SERVO_OFF, MAX_Z);
  XRemap = map_to(X, MIN_INPUT_READING, MAX_INPUT_READING, MIN_MAPPED_VALUE, MAX_MAPPED_VALUE);
  YRemap = map_to(Y, MIN_INPUT_READING, MAX_INPUT_READING, MIN_MAPPED_VALUE, MAX_MAPPED_VALUE);

  // Power given to each axis. xp, yp between 0 and 1
  xp = abs(XRemap - SERVO_OFF) / (MIN_MAPPED_VALUE - MAX_MAPPED_VALUE);
  yp = abs(YRemap - SERVO_OFF) / (MIN_MAPPED_VALUE - MAX_MAPPED_VALUE);

  valx1 = (3000 - XRemap) * (xp + 0.5 - yp);
  valy1 = YRemap * (yp + 0.5 - xp);
  valx2 = XRemap * (xp + 0.5 - yp);
  valy2 = (3000 - YRemap) * (yp + 0.5 - xp);
  
  // plan zone - diag
  YRemap2 = (float)(YRemap - SERVO_OFF);
  XRemap2 = (float)(XRemap - SERVO_OFF);
  if ((YRemap2 > (XRemap2 * 0.846) && YRemap2 < (XRemap2 * 1.182)) ||
      (YRemap2 > (XRemap2 * 1.182) && YRemap2 < (XRemap2 * 0.846)))
  {
    pwdValues["TYPE"] = 'A';
    pwdValues["FDX"] = SERVO_OFF;
    pwdValues["FSX"] = (int) (valy2 + valx2);
    pwdValues["RDX"] =  (int) (valy2 + valx1);
    pwdValues["RSX"] = SERVO_OFF;
  }
  else if (YRemap2 > (XRemap2 * 0.846 * (-1)) && YRemap2 < (XRemap2 * 1.182 * (-1)) ||
            (YRemap2 > (XRemap2 * 1.182 * (-1)) && YRemap2 < (XRemap2 * 0.846 * (-1)))) { 
    pwdValues["TYPE"] = 'A';
    pwdValues["FDX"] = SERVO_OFF;
    pwdValues["FDX"] = (int) (valy1 + valx2);
    pwdValues["FSX"] = SERVO_OFF;
    pwdValues["RDX"] = SERVO_OFF;
    pwdValues["RSX"] = (int) (valy2 + valx2);
  }
  else {
    pwdValues["TYPE"] = 'A';
    pwdValues["FDX"] = SERVO_OFF;
    pwdValues["FDX"] = (int) (valy1 + valx2);
    pwdValues["FSX"] = (int) (valy2 + valx2);
    pwdValues["RDX"] = (int) (valy2 + valx1);
    pwdValues["RSX"] = (int) (valy2 + valx2);
  }

  // Z-Axis control
  if (Z_URemap >= Z_DRemap) {
    pwdValues["TYPE"] = 'A';
    pwdValues["UPFDX"] = Z_URemap >= 1510 ? 3000 - Z_URemap : SERVO_OFF;
    pwdValues["UPRSX"] = Z_URemap >= 1510 ? 3000 - Z_URemap : SERVO_OFF;
    pwdValues["UPRDX"] = Z_URemap >= 1510 ? 3000 - Z_URemap : SERVO_OFF;
    pwdValues["UPFSX"] = Z_URemap >= 1510 ? 3000 - Z_URemap : SERVO_OFF;
  }
  else {
    pwdValues["TYPE"] = 'A';
    pwdValues["UPFDX"] = Z_DRemap >= 1510 ? Z_DRemap : SERVO_OFF;
    pwdValues["UPRSX"] = Z_DRemap >= 1510 ? Z_DRemap : SERVO_OFF;
    pwdValues["UPRDX"] = Z_DRemap >= 1510 ? Z_DRemap : SERVO_OFF;
    pwdValues["UPFSX"] = Z_DRemap >= 1510 ? Z_DRemap : SERVO_OFF;
  }
  
  changeControllerStatus(depth, Z_URemap);

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

void controlSystemCallFunction() {
  char DebugControllerInfo[200];
  
  if (control_on == true) {
    double forceZ = zControl.calculateZ(referenceZ, depth);
    double forcePitch = pitchControl.calculatePitch(referencePitch, depth);
    double forceRoll = rollControl.calculateRoll(referenceRoll, depth);
    OutputValues pwm = compute_PWM(forceZ, forceRoll, forcePitch);
    
    pwdValues["UPFDX"] = (int) pwm.T5;
    pwdValues["UPRSX"] = (int) pwm.T8;
    pwdValues["UPRDX"] = (int) pwm.T7;
    pwdValues["UPFSX"] = (int) pwm.T6;
    
    sprintf(DebugControllerInfo,
            "{\"refZ\":%f, \"Force(N)\":%f,\"depth\":%f,\"roll\":%f,\"pitch\":%f,\"Z_Uremap\":%d,\"Z_DRemap\":%d}",
            referenceZ,
            forceZ,
            depth,
            referenceRoll,
            referencePitch,
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

  if (Z_URemap >= 1550) {   //forse manca scendere
    referenceZ = depth;
    control_on = false; // !!!! wtf check !!!!
  }
}

void readSensorsData(){
  WT61P_read_angle();
	double referenceRoll_new = WT61P_get_roll();
	double referencePitch_new = WT61P_get_pitch();

  //if no connection read are zeros, so exclude them and keep the previous
  if (referenceRoll_new != 0)
    referenceRoll = referenceRoll_new;
  if (referencePitch_new != 0)
    referencePitch = referencePitch_new;
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
  }
  else{
    sleepMillis(20);
    fd = serialOpen("/dev/ttyACM1", 115200);
    if(fd>=0){
      std::cout << "serial /dev/ttyACM1 open" << std::endl;
    }
    else
      std::cout << "NO SERIAL CONNECTION" << std::endl;
  }
}

void my_handler(int s){
           printf("closing...\n");
           serialClose(fd);
           exit(1); 

}