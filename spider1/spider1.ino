#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>

/* =========================================================
   PCA9685 CONFIG
   ========================================================= */

Adafruit_PWMServoDriver pca(0x40);

const int I2C_SDA = 8;
const int I2C_SCL = 9;

const int SERVO_FREQ = 50;

const int SERVOMIN = 110;
const int SERVOMAX = 510;


/* =========================================================
   ROBOT STRUCTURE
   ========================================================= */

#define LEG_COUNT 4
#define JOINT_COUNT 3

enum Joint{
  COXA,
  FEMUR,
  TIBIA
};

inline int ch(int leg,int joint){
  return leg*3+joint;
}


/* =========================================================
   SERVO CONFIG
   ========================================================= */

int DIR[LEG_COUNT][JOINT_COUNT] = {
  {+1,+1,+1},
  {+1,+1,+1},
  {+1,+1,+1},
  {+1,+1,+1}
};

const int BASE_HOME = 90;


/* =========================================================
   ROBOT POSES
   ========================================================= */

int STAND_COXA[LEG_COUNT] = {
  -35,+35,-35,+35
};

int STAND_FEMUR = -45;
int STAND_TIBIA = 120;

int SLEEPLOW_FEMUR = 30;


/* =========================================================
   ROBOT STATE
   ========================================================= */

enum RobotState{
  STATE_SLEEP_LOW,
  STATE_START
};

RobotState currentState = STATE_SLEEP_LOW;


/* =========================================================
   ESP NOW COMMAND
   ========================================================= */

#define CMD_TOGGLE_STATE 1

typedef struct{
  uint8_t cmd;
}ControlPacket;


/* =========================================================
   SERVO FUNCTIONS
   ========================================================= */

int angleToPulse(int angle){

  angle = constrain(angle,0,180);
  return map(angle,0,180,SERVOMIN,SERVOMAX);
}

void writeAngle(int leg,int joint,int angle){

  pca.setPWM(ch(leg,joint),0,angleToPulse(angle));
}


/* =========================================================
   INITIAL POSE SETUP
   ========================================================= */

void initPose(){

  for(int leg=0;leg<LEG_COUNT;leg++){

    int coxa = BASE_HOME + DIR[leg][COXA]*STAND_COXA[leg];
    int tibia = BASE_HOME + DIR[leg][TIBIA]*STAND_TIBIA;
    int femur = BASE_HOME + DIR[leg][FEMUR]*SLEEPLOW_FEMUR;

    writeAngle(leg,COXA,coxa);
    writeAngle(leg,TIBIA,tibia);
    writeAngle(leg,FEMUR,femur);
  }

  delay(400);
}


/* =========================================================
   MOVE FEMUR ALL LEGS
   ========================================================= */

void moveFemurAll(int startOffset,int endOffset){

  int steps = 30;

  for(int i=0;i<=steps;i++){

    float t=(float)i/steps;
    float offset=startOffset+(endOffset-startOffset)*t;

    for(int leg=0;leg<LEG_COUNT;leg++){

      int femur = BASE_HOME + DIR[leg][FEMUR]*offset;

      writeAngle(leg,FEMUR,femur);
    }

    delay(35);
  }
}


/* =========================================================
   START POSE
   ========================================================= */

void goStartPose(){

  moveFemurAll(SLEEPLOW_FEMUR,STAND_FEMUR);
}


/* =========================================================
   SLEEP LOW
   ========================================================= */

void goSleepLowPose(){

  moveFemurAll(STAND_FEMUR,SLEEPLOW_FEMUR);
}


/* =========================================================
   STATE MACHINE
   ========================================================= */

void nextState(){

  switch(currentState){

    case STATE_SLEEP_LOW:

      currentState = STATE_START;
      goStartPose();
      break;

    case STATE_START:

      currentState = STATE_SLEEP_LOW;
      goSleepLowPose();
      break;
  }
}


/* =========================================================
   ESP NOW RECEIVE
   ========================================================= */

void onReceive(const esp_now_recv_info *info,
               const uint8_t *incomingData,
               int len){

  if(len!=sizeof(ControlPacket)) return;

  ControlPacket packet;

  memcpy(&packet,incomingData,sizeof(packet));

  if(packet.cmd==CMD_TOGGLE_STATE){

    nextState();
  }
}


/* =========================================================
   SETUP
   ========================================================= */

void setup(){

  Serial.begin(115200);

  Wire.begin(I2C_SDA,I2C_SCL);

  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  delay(20);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Spider MAC: ");
  Serial.println(WiFi.macAddress());

  if(esp_now_init()!=ESP_OK){

    Serial.println("ESP NOW INIT FAIL");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  initPose();
}


/* =========================================================
   LOOP
   ========================================================= */

void loop(){

}