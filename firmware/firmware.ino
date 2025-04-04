#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//#define DEBUG_INFO
//#define DEBUG_INFO1

Servo servos[12];  
MeMegaPiDCMotor dc;
MeTemperature ts;
MeRGBLed led;
MeUltrasonicSensor ultrasonic_sensor(PORT6);     //PORT_7
Me7SegmentDisplay seg;
MePort generalDevice;
MeLEDMatrix ledMx;
MeInfraredReceiver *ir = NULL;     //PORT_6
MeGyro gyro_ext(PORT7,0x68);           //external gryo sensor
MeCompass Compass;
MeJoystick joystick;
MeStepperOnBoard steppers[4] = {MeStepperOnBoard(1),MeStepperOnBoard(2),MeStepperOnBoard(3),MeStepperOnBoard(4)};
MeBuzzer buzzer;
MeHumiture humiture;
MeFlameSensor FlameSensor;
MeGasSensor GasSensor;
MeTouchSensor touchSensor;
Me4Button buttonSensor;
MeEncoderOnBoard* encoders[4] = {new MeEncoderOnBoard(SLOT1), new MeEncoderOnBoard(SLOT2), new MeEncoderOnBoard(SLOT3), new MeEncoderOnBoard(SLOT4)};
MeLineFollower line(PORT_8);
MeColorSensor *colorsensor  = NULL;

typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

union
{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
}val;

union
{
  uint8_t byteVal[8];
  double doubleVal;
}valDouble;

union
{
  uint8_t byteVal[2];
  int16_t shortVal;
}valShort;
MeModule modules[12];
#if defined(__AVR_ATmega32U4__) 
  int16_t analogs[12]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
#endif
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
  int16_t analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};
#endif
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
  int16_t analogs[16]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
#endif

int16_t len = 52;
int16_t servo_pins[12]={0,0,0,0,0,0,0,0,0,0,0,0};
//Just for MegaPi
int16_t moveSpeed = 200;
int16_t turnSpeed = 180;
int16_t minSpeed = 45;
int16_t factor = 23;
int16_t distance=0;
int16_t randnum = 0;                                                                               
int16_t LineFollowFlag=0;

#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define BALANCED_MODE                        0x02
#define IR_REMOTE_MODE                       0x03
#define LINE_FOLLOW_MODE                     0x04
#define MAX_MODE                             0x05

//#define POWER_PORT                           A4
//#define BUZZER_PORT                          45
//#define RGBLED_PORT                          44

#define DATA_SERIAL                            0
#define DATA_SERIAL1                           1
#define DATA_SERIAL2                           2
#define DATA_SERIAL3                           3

uint8_t command_index = 0;
uint8_t megapi_mode = BLUETOOTH_MODE;
uint8_t index = 0;
uint8_t dataLen;
uint8_t modulesLen=0;
uint8_t irRead = 0;
uint8_t prevc=0;
uint8_t BluetoothSource = DATA_SERIAL;
uint8_t keyPressed = KEY_NULL;
uint8_t serialRead;
uint8_t buffer[52];
uint8_t bufferBt1[52];
uint8_t bufferBt2[52];
double  lastTime = 0.0;
double  currentTime = 0.0;
double  CompAngleY, CompAngleX, GyroXangle;
double  LastCompAngleY, LastCompAngleX, LastGyroXangle;
double  last_turn_setpoint_filter = 0.0;
double  last_speed_setpoint_filter = 0.0;
double  last_speed_error_filter = 0.0;
double  speed_Integral_average = 0.0;
double  angle_speed = 0.0;

float angleServo = 90.0;
float dt;

long lasttime_angle = 0;
long lasttime_speed = 0;
long update_sensor = 0;
long blink_time = 0;
long last_Pulse_pos_encoder1 = 0;
long last_Pulse_pos_encoder2 = 0;

boolean isStart = false;
boolean isAvailable = false;
boolean leftflag;
boolean rightflag;
boolean start_flag = false;
boolean move_flag = false;
boolean blink_flag = false;

String mVersion = "0e.01.018";
//////////////////////////////////////////////////////////////////////////////////////
float RELAX_ANGLE = -1;                    //Natural balance angle,should be adjustment according to your own car
#define PWM_MIN_OFFSET   0

#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define TEMPERATURE_SENSOR     2
#define LIGHT_SENSOR           3
#define POTENTIONMETER         4
#define JOYSTICK               5
#define GYRO                   6
#define SOUND_SENSOR           7
#define RGBLED                 8
#define SEVSEG                 9
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define IR                     13
#define PIRMOTION              15
#define INFRARED               16
#define LINEFOLLOWER           17
#define SHUTTER                20
#define LIMITSWITCH            21
#define BUTTON                 22
#define HUMITURE               23
#define FLAMESENSOR            24
#define GASSENSOR              25
#define COMPASS                26
#define TEMPERATURE_SENSOR_1   27
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define TONE                   34
#define BUTTON_INNER           35
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define STEPPER                40
#define LEDMATRIX              41
#define TIMER                  50
#define TOUCH_SENSOR           51
#define JOYSTICK_MOVE          52
#define COMMON_COMMONCMD       60
  //Secondary command
  #define SET_STARTER_MODE     0x10
  #define SET_AURIGA_MODE      0x11
  #define SET_MEGAPI_MODE      0x12
  #define GET_BATTERY_POWER    0x70
  #define GET_AURIGA_MODE      0x71
  #define GET_MEGAPI_MODE      0x72
#define ENCODER_BOARD 61
  //Read type
  #define ENCODER_BOARD_POS    0x01
  #define ENCODER_BOARD_SPEED  0x02

#define ENCODER_PID_MOTION     62
  //Secondary command
  #define ENCODER_BOARD_POS_MOTION_MOVE    0x01
  #define ENCODER_BOARD_SPEED_MOTION       0x02
  #define ENCODER_BOARD_PWM_MOTION         0x03
  #define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
  #define ENCODER_BOARD_CAR_POS_MOTION     0x05
  #define ENCODER_BOARD_POS_MOTION_MOVETO  0x06

#define STEPPER_NEW            76
  //Secondary command
  #define STEPPER_POS_MOTION_MOVE          0x01
  #define STEPPER_SPEED_MOTION             0x02
  #define STEPPER_SET_CUR_POS_ZERO         0x04
  #define STEPPER_POS_MOTION_MOVETO        0x06

#define COLORSENSOR            67
  //Secondary command
  #define GETRGB                           0x01
  #define GETBOOL                          0x02
  #define GETCOLOR                         0x03

#define GET 1
#define RUN 2
#define RESET 4
#define START 5

/* Start of our Firmware Variables*/
#define TRANSLATION_CMD 0
#define ROTATION_CMD 1

#define ROTATION_BY_90_DEGREES_COUNTERCLOCKWISE 0
#define ROTATION_BY_90_DEGREES_CLOCKWISE 1
#define ROTATION_BY_180_DEGREES 2

#define MOVEMENT_BLOCK_SIZE_CM 30 // in cm

MeEncoderOnBoard *frontLeftEncoder = encoders[0];
MeEncoderOnBoard *rearRightEncoder = encoders[1];

int16_t frontLeftEncoderSpeed = -0.9 * moveSpeed;
int16_t rearRightEncoderSpeed = moveSpeed;

uint8_t bluetoothHeadA = 0xff;
uint8_t bluetoothHeadB = 0x55;
uint8_t bluetoothEnd = 0xff;

const float wheelDiameter = 6.4; //cm
const float wheelRadius = wheelDiameter / 2;
const float wheelCircumference = wheelDiameter * PI;
const int pulsesPerRevolution = 374; // also equals 360 degrees
const float distancePerPulse = wheelCircumference / pulsesPerRevolution;

uint8_t buf[64];
uint8_t bufindex;
unsigned long print_time = 0;
/* End of our Firmware Variables*/

typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral,differential, last_error;
} PID;

PID  PID_angle, PID_speed, PID_turn, PID_pos;

/**
 * \par Function
 *    encoder_move_finish_callback
 * \par Description
 *    This function called when encoder motor move finish.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void encoder_move_finish_callback(int slot,int extId)
{
  writeHead();
  writeSerial(extId);
  sendByte(slot);
  writeEnd();
}

/**
 * \par Function
 *    stepper_move_finish_callback
 * \par Description
 *    This function called when stepper motor move finish.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void stepper_move_finish_callback(int slot,int extId)
{
  writeHead();
  writeSerial(extId);
  sendByte(slot);
  writeEnd();
}
/**
 * \par Function
 *    isr_process_encoder1
 * \par Description
 *    This function use to process the interrupt of encoder1 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder1 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder1(void)
{
  if(digitalRead(encoders[0]->getPortB()) == 0)
  {
    encoders[0]->pulsePosMinus();
  }
  else
  {
    encoders[0]->pulsePosPlus();;
  }
}

/**
 * \par Function
 *    isr_process_encoder2
 * \par Description
 *    This function use to process the interrupt of encoder2 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder2 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder2(void)
{
  if(digitalRead(encoders[1]->getPortB()) == 0)
  {
    encoders[1]->pulsePosMinus();
  }
  else
  {
    encoders[1]->pulsePosPlus();
  }
}

/**
 * \par Function
 *    isr_process_encoder3
 * \par Description
 *    This function use to process the interrupt of encoder3 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder3 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder3(void)
{
  if(digitalRead(encoders[2]->getPortB()) == 0)
  {
    encoders[2]->pulsePosMinus();
  }
  else
  {
    encoders[2]->pulsePosPlus();
  }
}

/**
 * \par Function
 *    isr_process_encoder4
 * \par Description
 *    This function use to process the interrupt of encoder4 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder4 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder4(void)
{
  if(digitalRead(encoders[3]->getPortB()) == 0)
  {
    encoders[3]->pulsePosMinus();
  }
  else
  {
    encoders[3]->pulsePosPlus();
  }
}

/**
 * \par Function
 *    WriteBalancedDataToEEPROM
 * \par Description
 *    This function use to write the balanced car configuration parameters to EEPROM.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void WriteBalancedDataToEEPROM(void)
{
  EEPROM.write(BALANCED_CAR_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(BALANCED_CAR_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(BALANCED_CAR_START_ADDR, EEPROM_CHECK_START);

  EEPROM.put(BALANCED_CAR_NATURAL_BALANCE, RELAX_ANGLE);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR+4, PID_angle.I);
  EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR+8, PID_angle.D);

  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR+4, PID_speed.I);
  EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR+8, PID_speed.D);

  EEPROM.put(BALANCED_CAR_DIR_PID_ADDR, PID_turn.P);
  EEPROM.write(BALANCED_CAR_END_ADDR, EEPROM_CHECK_END);

  EEPROM.write(MEGAPI_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
  EEPROM.write(MEGAPI_MODE_END_ADDR, EEPROM_CHECK_END);
}

/**
 * \par Function
 *    WriteMegapiModeToEEPROM
 * \par Description
 *    This function use to write the MegaPi Mode configuration parameter to EEPROM.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void WriteMegapiModeToEEPROM(void)
{
  EEPROM.write(MEGAPI_MODE_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(MEGAPI_MODE_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(MEGAPI_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
  EEPROM.write(MEGAPI_MODE_END_ADDR, EEPROM_CHECK_END);
}

/**
 * \par Function
 *    readEEPROM
 * \par Description
 *    This function use to read the configuration parameters from EEPROM.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readEEPROM(void)
{
  if((EEPROM.read(BALANCED_CAR_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(BALANCED_CAR_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if((EEPROM.read(BALANCED_CAR_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(BALANCED_CAR_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(BALANCED_CAR_NATURAL_BALANCE, RELAX_ANGLE);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR+4, PID_angle.I);
      EEPROM.get(BALANCED_CAR_ANGLE_PID_ADDR+8, PID_angle.D);

      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR+4, PID_speed.I);
      EEPROM.get(BALANCED_CAR_SPEED_PID_ADDR+8, PID_speed.D);

      EEPROM.get(BALANCED_CAR_DIR_PID_ADDR, PID_turn.P);
#ifdef DEBUG_INFO
      Serial.println( "Read data from EEPROM:");
      Serial.print(RELAX_ANGLE);
      Serial.print( "  ");
      Serial.print(PID_angle.P);
      Serial.print( "  ");
      Serial.print(PID_angle.I);
      Serial.print( "  ");
      Serial.print(PID_angle.D);
      Serial.print( "  ");
      Serial.print(PID_speed.P);
      Serial.print( "  ");
      Serial.print(PID_speed.I);
      Serial.print( "  ");
      Serial.print(PID_speed.D);
      Serial.print( "  ");
      Serial.println(PID_turn.P);
#endif
    }
    else
    {
      Serial.println( "Data area damage on balanced car pid!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial.println( "First written Balanced data!" );
#endif
    WriteBalancedDataToEEPROM();
  }

  if((EEPROM.read(MEGAPI_MODE_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(MEGAPI_MODE_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if((EEPROM.read(MEGAPI_MODE_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(MEGAPI_MODE_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(MEGAPI_MODE_CONFIGURE, megapi_mode);
#ifdef DEBUG_INFO
      Serial.print( "Read megapi_mode from EEPROM:");
      Serial.println(megapi_mode);
#endif
    }
    else
    {
      Serial.println( "Data area damage on megapi mode!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial.println( "First written auriga mode!" );
#endif
    WriteMegapiModeToEEPROM();
  }
}

/**
 * \par Function
 *    BackwardAndTurnLeft
 * \par Description
 *    This function use to control the car kit go backward and turn left.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void BackwardAndTurnLeft(void)
{
  frontLeftEncoder->setMotorPwm(-moveSpeed/4);
  rearRightEncoder->setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    BackwardAndTurnRight
 * \par Description
 *    This function use to control the car kit go backward and turn right.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void BackwardAndTurnRight(void)
{
  frontLeftEncoder->setMotorPwm(-moveSpeed);
  rearRightEncoder->setMotorPwm(moveSpeed/4);
}

/**
 * \par Function
 *    TurnLeft1
 * \par Description
 *    This function use to control the car kit go backward and turn left(fast).
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnLeft1(void)
{
  frontLeftEncoder->setMotorPwm(moveSpeed);
  rearRightEncoder->setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    TurnRight1
 * \par Description
 *    This function use to control the car kit go backward and turn right(fast).
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnRight1(void)
{
  frontLeftEncoder->setMotorPwm(-moveSpeed);
  rearRightEncoder->setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    Stop
 * \par Description
 *    This function use to stop the car kit.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Stop(void)
{
  frontLeftEncoder->setMotorPwm(0);
  rearRightEncoder->setMotorPwm(0);
}

/**
 * \par Function
 *    ChangeSpeed
 * \par Description
 *    This function use to change the speed of car kit.
 * \param[in]
 *    spd - the speed of car kit(-255 ~ 255)
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void ChangeSpeed(int16_t spd)
{
  moveSpeed = spd;
}

/**
 * \par Function
 *    readBuffer
 * \par Description
 *    This function use to read the serial data from its buffer..
 * \param[in]
 *    index - The first address in the array
 * \par Output
 *    None
 * \return
 *    The data need to be read.
 * \par Others
 *    None
 */
uint8_t readBuffer(int16_t index)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    return buffer[index];
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    return bufferBt1[index];
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    return bufferBt2[index];
  }
}

/**
 * \par Function
 *    writeBuffer
 * \par Description
 *    This function use to write the serial data to its buffer..
 * \param[in]
 *    index - The data's first address in the array
  * \param[in]
 *    c - The data need to be write.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeBuffer(int16_t index,uint8_t c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    buffer[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    bufferBt1[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    bufferBt2[index]=c;
  }
}

/**
 * \par Function
 *    writeHead
 * \par Description
 *    This function use to write the head of transmission frame.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeHead(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
}

/**
 * \par Function
 *    writeEnd
 * \par Description
 *    This function use to write the terminator of transmission frame.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeEnd(void)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.println();
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.println();
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.println();
  }
}

/**
 * \par Function
 *    writeSerial
 * \par Description
 *    This function use to write the data to serial.
 * \param[in]
 *    c - The data need to be write.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeSerial(uint8_t c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.write(c);
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.write(c);
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.write(c);
  }
}

/**
 * \par Function
 *    readSerial
 * \par Description
 *    This function use to read the data from serial, and fill the data
 *    to its buffer.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readSerial(void)
{
  isAvailable = false;
  if(Serial.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL;
    serialRead = Serial.read();
  }
  else if(Serial2.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL2;
    serialRead = Serial2.read();
  }
  else if(Serial3.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL3;
    serialRead = Serial3.read();
  }
}

/**
 * \par Function
 *    parseData
 * \par Description
 *    This function use to process the data from the serial port,
 *    call the different treatment according to its action.
 *    ff 55 len idx action device port  slot  data a
 *    0  1  2   3   4      5      6     7     8
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void parseData(void)
{
  isStart = false;
  uint8_t idx = readBuffer(3);
  uint8_t action = readBuffer(4);
  uint8_t device = readBuffer(5);
  command_index = (uint8_t)idx;
  switch(action)
  {
    case GET:
      {
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN:
      {
        runModule(device);
        callOK();
      }
      break;
    case RESET:
      {
        //reset
        /* reset On-Board encoder driver */
        for(int i=0;i<4;i++)
        {
          encoders[i]->setPulsePos(0);
          encoders[i]->moveTo(0,10);
          encoders[i]->setMotorPwm(0);
          encoders[i]->setMotionMode(DIRECT_MODE);
          steppers[i].setCurrentPosition(0);
          steppers[i].moveTo(0);
          steppers[i].disableOutputs();
        }

        /* reset dc motor on driver port */
        dc.reset(PORT1A);
        dc.run(0);
        dc.reset(PORT1B);
        dc.run(0);
        dc.reset(PORT2A);
        dc.run(0);
        dc.reset(PORT2B);
        dc.run(0);
        dc.reset(PORT3A);
        dc.run(0);
        dc.reset(PORT3B);
        dc.run(0);
        dc.reset(PORT4A);
        dc.run(0);
        dc.reset(PORT4B);
        dc.run(0);

        /* reset stepper motor driver */
        
        callOK();
      }
      break;
     case START:
      {
        //start
        callOK();
      }
      break;
  }
}

/**
 * \par Function
 *    callOK
 * \par Description
 *    Response for executable commands.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void callOK(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

/**
 * \par Function
 *    sendByte
 * \par Description
 *    Send byte data
 * \param[in]
 *    c - the byte data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendByte(uint8_t c)
{
  writeSerial(1);
  writeSerial(c);
}

/**
 * \par Function
 *    sendString
 * \par Description
 *    Send string data
 * \param[in]
 *    s - the string data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendString(String s)
{
  int16_t l = s.length();
  for(int16_t i=0;i<l;i++)
  {
    writeSerial(s.charAt(i));
  }
}

/**
 * \par Function
 *    sendFloat
 * \par Description
 *    Sned float data
 * \param[in]
 *    value - the float data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendFloat(float value)
{ 
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
 * \par Function
 *    sendLong
 * \par Description
 *    Sned long data
 * \param[in]
 *    value - the long data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendLong(long value)
{ 
  writeSerial(6);
  val.longVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
 * \par Function
 *    sendShort
 * \par Description
 *    Sned short data
 * \param[in]
 *    value - the short data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendShort(int16_t value)
{
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

/**
 * \par Function
 *    sendDouble
 * \par Description
 *    Sned double data, same as float data on arduino.
 * \param[in]
 *    value - the double data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendDouble(double value)
{
  writeSerial(5);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}

/**
 * \par Function
 *    readShort
 * \par Description
 *    read the short data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the short data.
 * \par Others
 *    None
 */
int16_t readShort(int16_t idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}

/**
 * \par Function
 *    readFloat
 * \par Description
 *    read the float data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the float data.
 * \par Others
 *    None
 */
float readFloat(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}

/**
 * \par Function
 *    readLong
 * \par Description
 *    read the long data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the long data.
 * \par Others
 *    None
 */
long readLong(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.longVal;
}

char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};

/**
 * \par Function
 *    readString
 * \par Description
 *    read the string data.
 * \param[in]
 *    idx - The string's first address in the array.
 * \param[in]
 *    len - The length of the string data.
 * \par Output
 *    None
 * \return
 *    the address of string data.
 * \par Others
 *    None
 */
char* readString(int16_t idx,int16_t len)
{
  for(int16_t i=0;i<len;i++)
  {
    _receiveStr[i]=readBuffer(idx+i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}

/**
 * \par Function
 *    readUint8
 * \par Description
 *    read the uint8 data.
 * \param[in]
 *    idx - The Uint8 data's first address in the array.
 * \param[in]
 *    len - The length of the uint8 data.
 * \par Output
 *    None
 * \return
 *    the address of uint8 data.
 * \par Others
 *    None
 */
uint8_t* readUint8(int16_t idx,int16_t len)
{
  for(int16_t i=0;i<len;i++)
  {
    if(i > 15)
    {
      break;
    }
    _receiveUint8[i] = readBuffer(idx+i);
  }
  return _receiveUint8;
}

/**
 * \par Function
 *    initStepper
 * \par Description
 *    Initialize acceleration, subdivision, and speed for stepper motor.
 * \param[in]
 *    index - The index of stepper.
 * \param[in]
 *    maxSpeed - The max speed of stepper.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void initStepper(uint8_t index,int16_t maxSpeed)
{
  // steppers[index].setpin(index+1);

  steppers[index].setMaxSpeed(maxSpeed);
  steppers[index].setAcceleration(20000);
  steppers[index].setMicroStep(16);
  steppers[index].setSpeed(maxSpeed);
  steppers[index].enableOutputs();
}
/**
 * \par Function
 *    runModule
 * \par Description
 *    Processing execute commands.
 * \param[in]
 *    device - The definition of all execute commands.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void runModule(uint8_t device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  uint8_t port = readBuffer(6);
  uint8_t pin = port;
  switch(device)
  {
    case MOTOR:
      {
        int16_t speed = readShort(7);
        dc.reset(port);
        dc.run(speed);
      }
      break;
    case ENCODER_BOARD:
      if(port == 0)
      {
        uint8_t slot = readBuffer(7);
        int16_t speed_value = readShort(8);
        speed_value = -speed_value;
        encoders[slot-1]->setTarPWM(speed_value);
      }
      break;
    case JOYSTICK:
      {
        int16_t leftSpeed = readShort(6);
        encoders[0]->setTarPWM(-leftSpeed);
        int16_t rightSpeed = readShort(8);
        encoders[1]->setTarPWM(-rightSpeed);
      }
      break;
    case STEPPER_NEW:
      {
        uint8_t subcmd = port;
        uint8_t extID = readBuffer(3);
        uint8_t slot_num = readBuffer(7);
        int16_t maxSpeed = 0;
        if(STEPPER_POS_MOTION_MOVE == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          maxSpeed = abs(speed_temp);

          initStepper(slot_num - 1,maxSpeed);
          steppers[slot_num - 1].move(pos_temp,extID,stepper_move_finish_callback);
        }
        if(STEPPER_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);

          initStepper(slot_num - 1,speed_temp);
          steppers[slot_num - 1].setSpeed(speed_temp);
        }
        if(STEPPER_POS_MOTION_MOVETO == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          maxSpeed = abs(speed_temp);

          initStepper(slot_num - 1,maxSpeed);
          steppers[slot_num - 1].moveTo(pos_temp,extID,stepper_move_finish_callback);
        }
        else if(STEPPER_SET_CUR_POS_ZERO == subcmd)
        {
          steppers[slot_num - 1].setCurrentPosition(0);
        }
      }
      break;
    case STEPPER:
      {
        int16_t maxSpeed = readShort(7);
        long distance = readLong(9);
        steppers[port - 1] = MeStepperOnBoard(port);
        initStepper(port - 1,maxSpeed);
        steppers[port - 1].moveTo(distance);
      } 
      break;
    case RGBLED:
      {
        uint8_t slot = readBuffer(7);
        uint8_t idx = readBuffer(8);
        uint8_t pixels_len = readBuffer(2) - 6;
        if((port != 0) && ((led.getPort() != port) || (led.getSlot() != slot)))
        {
          led.reset(port,slot);
        }
        if(idx>0)
        {
          for(uint8_t i=0;i<pixels_len;i+=3)
          {
            led.setColorAt(idx+i/3-1,readBuffer(9+i),readBuffer(10+i),readBuffer(11+i)); 
          }
        }
        else
        {
          led.setColor(readBuffer(9),readBuffer(10),readBuffer(11)); 
        }
        led.show();
      }
      break;
    case COMMON_COMMONCMD:
      {
        uint8_t subcmd = port;
        uint8_t cmd_data = readBuffer(7);
        if(SET_MEGAPI_MODE == subcmd)
        {
          Stop();
          if((cmd_data == BALANCED_MODE) || 
             (cmd_data == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE) || 
             (cmd_data == BLUETOOTH_MODE) ||
             (cmd_data == IR_REMOTE_MODE) ||
             (cmd_data == LINE_FOLLOW_MODE))
          {
            megapi_mode = cmd_data;
            if(EEPROM.read(MEGAPI_MODE_CONFIGURE) != megapi_mode)
            {
              EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
            }
          }
          else
          {
            megapi_mode = BLUETOOTH_MODE;
            if(EEPROM.read(MEGAPI_MODE_CONFIGURE) != megapi_mode)
            {
              EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
            }
          }
        }
      }
      break;
    case SERVO:
      {
        uint8_t slot = readBuffer(7);
        pin = slot==1?mePort[port].s1:mePort[port].s2;
        uint8_t v = readBuffer(8);
        Servo sv = servos[searchServoPin(pin)];
        if(v >= 0 && v <= 180)
        {
          if(!sv.attached())
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case SEVSEG:
      {
        if(seg.getPort() != port)
        {
          seg.reset(port);
        }
        float v = readFloat(7);
        seg.display(v);
      }
      break;
    case LEDMATRIX:
      {
        if(ledMx.getPort()!=port)
        {
          ledMx.reset(port);
        }
        uint8_t action = readBuffer(7);
        if(action==1)
        {
          int8_t px = readBuffer(8);
          int8_t py = readBuffer(9);
          int8_t len = readBuffer(10);
          char *s = readString(11,len);
          ledMx.drawStr(px,py,s);
        }
        else if(action==2)
        {
          int8_t px = readBuffer(8);
          int8_t py = readBuffer(9);
          uint8_t *ss = readUint8(10,16);
          ledMx.drawBitmap(px,py,16,ss);
        }
        else if(action==3)
        {
          int8_t point = readBuffer(8);
          int8_t hours = readBuffer(9);
          int8_t minutes = readBuffer(10);
          ledMx.showClock(hours,minutes,point);
        }
        else if(action == 4)
        {
          ledMx.showNum(readFloat(8),3);
        }
      }
      break;
    case LIGHT_SENSOR:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        uint8_t v = readBuffer(7);
        generalDevice.dWrite1(v);
      }
      break;
    case SHUTTER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        uint8_t v = readBuffer(7);
        if(v < 2)
        {
          generalDevice.dWrite1(v);
        }
        else
        {
          generalDevice.dWrite2(v-2);
        }
      }
      break;
    case DIGITAL:
      {
        pinMode(pin,OUTPUT);
        uint8_t v = readBuffer(7);
        digitalWrite(pin,v);
     }
     break;
    case PWM:
      {
        pinMode(pin,OUTPUT);
        uint8_t v = readBuffer(7);
        analogWrite(pin,v);
      }
      break;
    case SERVO_PIN:
      {
        uint8_t v = readBuffer(7);
        if(v >= 0 && v <= 180)
        {
          Servo sv = servos[searchServoPin(pin)];
          if(!sv.attached())
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case TIMER:
      {
        lastTime = millis()/1000.0; 
      }
      break;
    case JOYSTICK_MOVE:
      {
        if(port == 0)
        {
           int16_t joy_x = readShort(7);
           int16_t joy_y = readShort(9);
           double joy_x_temp = (double)joy_x * 0.2;    //0.3
           double joy_y_temp = -(double)joy_y * 0.15;  //0.2
           PID_speed.Setpoint = joy_y_temp;
           PID_turn.Setpoint = joy_x_temp;
           if(abs(PID_speed.Setpoint) > 1)
           { 
             move_flag = true;
           }
        }
      }
      break;
    case ENCODER_PID_MOTION:
      {
        uint8_t subcmd = port;
        uint8_t extID = readBuffer(3);
        uint8_t slot_num = readBuffer(7);
        if(ENCODER_BOARD_POS_MOTION_MOVE == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          encoders[slot_num-1]->move(pos_temp,(float)speed_temp,extID,encoder_move_finish_callback);
        }
        if(ENCODER_BOARD_POS_MOTION_MOVETO == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          encoders[slot_num-1]->moveTo(pos_temp,(float)speed_temp,extID,encoder_move_finish_callback);
        }
        else if(ENCODER_BOARD_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);  
          encoders[slot_num-1]->runSpeed((float)speed_temp);
        }
        else if(ENCODER_BOARD_PWM_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);  
          encoders[slot_num-1]->setTarPWM(speed_temp);     
        }
        else if(ENCODER_BOARD_SET_CUR_POS_ZERO == subcmd)
        {
          encoders[slot_num-1]->setPulsePos(0);     
        }
        else if(ENCODER_BOARD_CAR_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          if(slot_num == 1)
          {
            encoders[0]->move(pos_temp,(float)speed_temp);
            encoders[1]->move(-pos_temp,(float)speed_temp);
          }
          else if(slot_num == 2)
          {
            encoders[0]->move(-pos_temp,(float)speed_temp);
            encoders[1]->move(pos_temp,(float)speed_temp);
          }
          else if(slot_num == 3)
          {
            encoders[0]->move(pos_temp,(float)speed_temp);
            encoders[1]->move(pos_temp,(float)speed_temp);
          }
          else if(slot_num == 4)
          {
            encoders[0]->move(-pos_temp,(float)speed_temp);
            encoders[1]->move(-pos_temp,(float)speed_temp);
          }
        }
      }
      break;
  }
}

/**
 * \par Function
 *    searchServoPin
 * \par Description
 *    Check if the pin has been allocated, if it is not allocated,
 *    then allocate it.
 * \param[in]
 *    pin - arduino gpio number
 * \par Output
 *    None
 * \return
 *    the servo number be assigned
 * \par Others
 *    None
 */
int16_t searchServoPin(int16_t pin)
{
  for(uint8_t i=0;i<12;i++)
  {
    if(servo_pins[i] == pin)
    {
      return i;
    }
    if(servo_pins[i] == 0)
    {
      servo_pins[i] = pin;
      return i;
    }
  }
  return 0;
}
/**
 * \par Function
 *    readSensor
 * \par Description
 *    This function is used to process query command.
 * \param[in]
 *    device - The definition of all query commands.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readSensor(uint8_t device)
{
  /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value=0.0;
  uint8_t port,slot,pin;
  port = readBuffer(6);
  pin = port;
  writeHead();
  writeSerial(command_index);
  switch(device)
  {
    case ULTRASONIC_SENSOR:
      {
        if(ultrasonic_sensor.getPort() != port)
        {
          ultrasonic_sensor = MeUltrasonicSensor(port);
        }
        value = (float)ultrasonic_sensor.distanceCm();
        sendFloat(value);
      }
      break;
    case TEMPERATURE_SENSOR:
      {
        slot = readBuffer(7);
        if(ts.getPort() != port || ts.getSlot() != slot)
        {
          ts.reset(port,slot);
        }
        value = ts.temperature();
        sendFloat(value);
      }
      break;
    case LIGHT_SENSOR:
    case SOUND_SENSOR:
    case POTENTIONMETER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.aRead2();
        sendFloat(value);
      }
      break;
    case JOYSTICK:
      {
        slot = readBuffer(7);
        if(joystick.getPort() != port)
        {
          joystick.reset(port);
        }
        if(slot==0)
        {
          sendShort(joystick.read(1));
          sendShort(joystick.read(2));
        }
        else
        {
          value = joystick.read(slot);
          sendFloat(value);
        }
      }
      break;
    case INFRARED:
      {
        if(ir == NULL)
        {
          ir = new MeInfraredReceiver(port);
          ir->begin();
        }
        else if(ir->getPort() != port)
        {
          delete ir;
          ir = new MeInfraredReceiver(port);
          ir->begin();
        }
        irRead = ir->getCode();
        if((irRead < 255) && (irRead > 0))
        {
          sendFloat((float)irRead);
        }
        else
        {
          sendFloat(0);
        }
      }
      break;
    case PIRMOTION:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case LINEFOLLOWER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin1(),INPUT);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.dRead1()*2+generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case LIMITSWITCH:
      {
        slot = readBuffer(7);
        if(generalDevice.getPort() != port || generalDevice.getSlot() != slot)
        {
          generalDevice.reset(port,slot);
        }
        if(slot == 1)
        {
          pinMode(generalDevice.pin1(),INPUT_PULLUP);
          value = !generalDevice.dRead1();
        }
        else
        {
          pinMode(generalDevice.pin2(),INPUT_PULLUP);
          value = !generalDevice.dRead2();
        }
        sendFloat(value);  
      }
      break;
    case COMPASS:
      {
        if(Compass.getPort() != port)
        {
          Compass.reset(port);
          Compass.setpin(Compass.pin1(),Compass.pin2());
        }
        double CompassAngle;
        CompassAngle = Compass.getAngle();
        sendFloat((float)CompassAngle);
      }
      break;
    case HUMITURE:
      {
        uint8_t index = readBuffer(7);
        if(humiture.getPort() != port)
        {
          humiture.reset(port);
        }
        uint8_t HumitureData;
        humiture.update();
        if(index==2){
          sendByte(humiture.getValue(0));
          sendByte(humiture.getValue(1));
        }else{
          HumitureData = humiture.getValue(index);
          sendByte(HumitureData);
        }
      }
      break;
    case FLAMESENSOR:
      {
        if(FlameSensor.getPort() != port)
        {
          FlameSensor.reset(port);
          FlameSensor.setpin(FlameSensor.pin2(),FlameSensor.pin1());
        }
        int16_t FlameData; 
        FlameData = FlameSensor.readAnalog();
        sendShort(FlameData);
      }
      break;
    case GASSENSOR:
      {
        if(GasSensor.getPort() != port)
        {
          GasSensor.reset(port);
          GasSensor.setpin(GasSensor.pin2(),GasSensor.pin1());
        }
        int16_t GasData; 
        GasData = GasSensor.readAnalog();
        sendShort(GasData);
      }
      break;
    case GYRO:
      {
        uint8_t axis = readBuffer(7);
        if((port == 0) && (gyro_ext.getDevAddr() == 0x68))      //extern gyro
        {
          if(axis==0)
          {
            sendFloat(gyro_ext.getAngle(1));
            sendFloat(gyro_ext.getAngle(2));
            sendFloat(gyro_ext.getAngle(3));
          }
          else
          {
            sendFloat(gyro_ext.getAngle(axis));
          }
        }
        else
        {
          sendFloat(0);
        }
      }
      break;
    case COLORSENSOR:
      {
        uint8_t colorsubcmd = 0;
        uint8_t colorindex = 0;
        uint8_t result = 0;
        uint32_t rgbcode = 0;
     
        colorsubcmd = readBuffer(7);
        colorindex  = readBuffer(8);

        if(colorsensor == NULL)
        {
          colorsensor = new MeColorSensor(port);
        }
        else if(colorsensor->getPort() != port)
        {
          delete colorsensor;
          colorsensor = new MeColorSensor(port);
        }
        
        if(colorsubcmd == GETRGB)
        {
          if(colorindex == 0x00)//r
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            result = (uint8_t)(rgbcode>>16);
          }
          else if(colorindex == 0x01)//g
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            result = (uint8_t)(rgbcode>>8);
          }
          else if(colorindex == 0x02)//b
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            result = (uint8_t)rgbcode;
          }
          else if(colorindex == 0x03)//rgb
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            sendLong(rgbcode);
          }
        }
        else if(colorsubcmd == GETBOOL)
        {
          result = colorsensor->ColorIdentify();
          if(colorindex == 0x00)
          {
            if(result == WHITE)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x02)
          {
            if(result == RED)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x04)
          {
            if(result == YELLOW)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x05)
          {
            if(result == GREEN)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x07)
          {
            if(result == BLUE)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x09)
          {
            if(result == BLACK)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
        }
        sendByte(result);
      }
      break;
    case VERSION:
      {
        sendString(mVersion);
      }
      break;
    case DIGITAL:
      {
        pinMode(pin,INPUT);
        sendFloat(digitalRead(pin));
      }
      break;
    case ANALOG:
      {
        pin = analogs[pin];
        pinMode(pin,INPUT);
        sendFloat(analogRead(pin));
      }
      break;
    case PULSEIN:
      {
        int16_t pw = readShort(7);
        pinMode(pin, INPUT);
        sendLong(pulseIn(pin,HIGH,pw));
      }
      break;
    case ULTRASONIC_ARDUINO:
      {
        uint8_t trig = readBuffer(6);
        uint8_t echo = readBuffer(7);
        long pw_data;
        float dis_data;
        pinMode(trig,OUTPUT);
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW);
        pinMode(echo, INPUT);
        pw_data = pulseIn(echo,HIGH,30000);
        dis_data = pw_data/58.0;
        delay(5);
        writeHead();
        writeSerial(command_index);
        sendFloat(pw_data);
      }
      break;
    case TIMER:
      {
        sendFloat((float)currentTime);
      }
      break;
    case TOUCH_SENSOR:
      {
        if(touchSensor.getPort() != port)
        {
          touchSensor.reset(port);
        }
        sendByte(touchSensor.touched());
      }
      break;
    case BUTTON:
      {

        uint8_t key_num = readBuffer(7);
        if(buttonSensor.getPort() != port)
        {
          buttonSensor.reset(port);
        }

        if(key_num == 0)
        {
          sendByte(keyPressed);
        }
        else
        {
          sendByte(keyPressed == readBuffer(7));
        }
      }
      break;
    case ENCODER_BOARD:
      {
        if(port == 0)
        {
          slot = readBuffer(7);
          uint8_t read_type = readBuffer(8);
          if(read_type == ENCODER_BOARD_POS)
          {
            sendLong(encoders[slot-1]->getCurPos());
          }
          else if(read_type == ENCODER_BOARD_SPEED)
          {
            sendFloat(encoders[slot-1]->getCurrentSpeed());
          }
        }
      }
      break;
    case COMMON_COMMONCMD:
      {
        uint8_t subcmd = port;
        if(GET_MEGAPI_MODE == subcmd)
        {
          sendByte(megapi_mode);
        }
      }
      break;
    default:
      {
        sendFloat(0);
      }
      break;
  }//switch
}

/**
 * \par Function
 *    PID_angle_compute
 * \par Description
 *    The angle process for balance car
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void PID_angle_compute(void)   //PID
{
  CompAngleX = -gyro_ext.getAngleX();
  double error = CompAngleX - PID_angle.Setpoint;
  PID_angle.Integral += error;
  PID_angle.Integral = constrain(PID_angle.Integral,-100,100); 
  PID_angle.differential = angle_speed;
  PID_angle.Output = PID_angle.P * error + PID_angle.I * PID_angle.Integral + PID_angle.D * PID_angle.differential;
  if(PID_angle.Output > 0)
  {
    PID_angle.Output = PID_angle.Output + PWM_MIN_OFFSET;
  }
  else
  {
    PID_angle.Output = PID_angle.Output - PWM_MIN_OFFSET;
  }

  double pwm_left = PID_angle.Output - PID_turn.Output;
  double pwm_right = -PID_angle.Output - PID_turn.Output;

#ifdef DEBUG_INFO
  Serial.print("Relay: ");
  Serial.print(PID_angle.Setpoint);
  Serial.print(" AngX: ");
  Serial.print(CompAngleX);
  Serial.print(" Output: ");
  Serial.print(PID_angle.Output);
  Serial.print(" PID_angle.Integral: ");
  Serial.print(PID_angle.Integral);
  Serial.print(" dif: ");
  Serial.println(PID_angle.differential);
#endif

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  encoders[0]->setMotorPwm(pwm_left);
  encoders[1]->setMotorPwm(pwm_right);
}

/**
 * \par Function
 *    PID_speed_compute
 * \par Description
 *    The speed process for balance car
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void PID_speed_compute(void)
{
  double speed_now = (encoders[1]->getCurrentSpeed() - encoders[0]->getCurrentSpeed())/2;

  last_speed_setpoint_filter  = last_speed_setpoint_filter  * 0.8;
  last_speed_setpoint_filter  += PID_speed.Setpoint * 0.2;
 
  if((move_flag == true) && (abs(speed_now) < 8) && (PID_speed.Setpoint == 0))
  {
    move_flag = false;
    last_speed_setpoint_filter = 0;
    PID_speed.Integral = speed_Integral_average;
  }

  double error = speed_now - last_speed_setpoint_filter;
  PID_speed.Integral += error;

  if(move_flag == true) 
  { 
    PID_speed.Integral = constrain(PID_speed.Integral , -2000, 2000);
    PID_speed.Output = PID_speed.P * error + PID_speed.I * PID_speed.Integral;
    PID_speed.Output = constrain(PID_speed.Output , -8.0, 8.0);
  }
  else
  {  
    PID_speed.Integral = constrain(PID_speed.Integral , -2000, 2000);
    PID_speed.Output = PID_speed.P * speed_now + PID_speed.I * PID_speed.Integral;
    PID_speed.Output = constrain(PID_speed.Output , -10.0, 10.0);
    speed_Integral_average = 0.8 * speed_Integral_average + 0.2 * PID_speed.Integral;
  }
  
#ifdef DEBUG_INFO
  Serial.print("Speed now: ");
  Serial.print(speed_now);
  Serial.print(", PID_speed.Setpoint: ");
  Serial.print(PID_speed.Setpoint);
  Serial.print(", Last Speed Error: ");     
  Serial.print(last_speed_error_filter);
  Serial.print(", Last Speed Setpoint Filter: ");
  Serial.print(last_speed_setpoint_filter);
  Serial.print(", PID_speed.Integral: ");
  Serial.print(PID_speed.Integral);
  Serial.print(", PID_speed.Output: ");
  Serial.println(PID_speed.Output);
#endif
  PID_angle.Setpoint =  RELAX_ANGLE + PID_speed.Output;
}

int16_t agx_start_count;

/**
 * \par Function
 *    reset
 * \par Description
 *    The exception process for balance car
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void reset(void)
{
  if((start_flag == false) && (abs(gyro_ext.getAngleX()) < 5))
  {
    agx_start_count++;
  }
  if((start_flag == true) && (abs(gyro_ext.getAngleX()) > 32))
  {
    agx_start_count = 0;
    encoders[0]->setMotorPwm(0);
    encoders[1]->setMotorPwm(0);
    PID_speed.Integral = 0;
    PID_angle.Setpoint = RELAX_ANGLE;
    PID_speed.Setpoint = 0;
    PID_turn.Setpoint = 0;
    encoders[0]->setPulsePos(0);
    encoders[1]->setPulsePos(0);
    PID_speed.Integral = 0;
    start_flag = false;
    last_speed_setpoint_filter = 0.0;
    last_turn_setpoint_filter = 0.0;
#ifdef DEBUG_INFO
    Serial.println("> 32");
#endif
  }
  else if(agx_start_count > 20)
  {
    agx_start_count = 0;
    PID_speed.Integral = 0;
    encoders[0]->setMotorPwm(0);
    encoders[1]->setMotorPwm(0);
    PID_angle.Setpoint = RELAX_ANGLE;
    encoders[0]->setPulsePos(0);
    encoders[1]->setPulsePos(0);
    lasttime_speed = lasttime_angle = millis();
    start_flag = true;
#ifdef DEBUG_INFO
    Serial.println("< 5");
#endif
  }
}

/**
 * \par Function
 *    parseGcode
 * \par Description
 *    The function used to configure parameters for balance car.
 * \param[in]
 *    cmd - Gcode command
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void parseGcode(char * cmd)
{
  char * tmp;
  char * str;
  char g_code_cmd;
  float p_value = 0;
  float i_value = 0;
  float d_value = 0;
  float relax_angle = 0;
  str = strtok_r(cmd, " ", &tmp);
  g_code_cmd = str[0];
  while(str!=NULL)
  {
    str = strtok_r(0, " ", &tmp);
    if((str[0]=='P') || (str[0]=='p')){
      p_value = atof(str+1);
    }else if((str[0]=='I') || (str[0]=='i')){
      i_value = atof(str+1);
    }else if((str[0]=='D') || (str[0]=='d')){
      d_value = atof(str+1);
    }
    else if((str[0]=='Z') || (str[0]=='z')){
      relax_angle  = atof(str+1);
    }
    else if((str[0]=='M') || (str[0]=='m')){
      megapi_mode  = atof(str+1);
    }
  }
//#ifdef DEBUG_INFO
  Serial.print("PID: ");
  Serial.print(p_value);
  Serial.print(", ");
  Serial.print(i_value);
  Serial.print(",  ");
  Serial.println(d_value);
//#endif
  if(g_code_cmd == '1')
  {
    PID_angle.P = p_value;
    PID_angle.I = i_value;
    PID_angle.D = d_value;
    EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR, PID_angle.P);
    EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR+4, PID_angle.I);
    EEPROM.put(BALANCED_CAR_ANGLE_PID_ADDR+8, PID_angle.D);
  }
  else if(g_code_cmd == '2')
  {
    PID_speed.P = p_value;
    PID_speed.I = i_value;
    PID_speed.D = d_value;
    EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR, PID_speed.P);
    EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR+4, PID_speed.I);
    EEPROM.put(BALANCED_CAR_SPEED_PID_ADDR+8, PID_speed.D);
  }
  else if(g_code_cmd == '3')
  {
    RELAX_ANGLE = relax_angle;
    EEPROM.put(BALANCED_CAR_NATURAL_BALANCE, relax_angle);
  }
  else if(g_code_cmd == '4')
  {
    if(EEPROM.read(MEGAPI_MODE_CONFIGURE) != megapi_mode)
    {
      EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
    }
    Serial.print("megapi_mode: ");
    Serial.println(megapi_mode);
  }
}

/**
 * \par Function
 *    parseCmd
 * \par Description
 *    The function used to parse Gcode command.
 * \param[in]
 *    cmd - Gcode command
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void parseCmd(char * cmd)
{
  if((cmd[0]=='g') || (cmd[0]=='G'))
  { 
    // gcode
    parseGcode(cmd+1);
  }
}

/**
 * \par Function
 *    balanced_model
 * \par Description
 *    The main function for balanced car model
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void balanced_model(void)
{
  reset();
  if(start_flag == true)
  {
    if((millis() - lasttime_angle) > 10)
    {
      PID_angle_compute();
      lasttime_angle = millis();
    }    
    if((millis() - lasttime_speed) > 100)
    {
      PID_speed_compute();
      last_turn_setpoint_filter  = last_turn_setpoint_filter * 0.8;
      last_turn_setpoint_filter  += PID_turn.Setpoint * 0.2;
      PID_turn.Output = last_turn_setpoint_filter;
      lasttime_speed = millis();
    }
  }
  else
  {
    encoders[0]->setMotorPwm(0);
    encoders[1]->setMotorPwm(0);
  } 
}

/**
 * \par Function
 *    ultrCarProcess
 * \par Description
 *    The main function for ultrasonic automatic obstacle avoidance
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void ultrCarProcess(void)
{
  moveSpeed = 150;
  if(ultrasonic_sensor.getPort() > 0)
  {
    distance = ultrasonic_sensor.distanceCm();
  }
  else
  {
    return;
  }

  if((distance > 20) && (distance < 40))
  {
    randnum=random(300);
    if((randnum > 190) && (!rightflag))
    {
      leftflag=true;
      TurnLeft();
    }
    else
    {
      rightflag=true;
      TurnRight();  
    }
  }
  else if((distance < 20) && (distance > 0))
  {
    randnum=random(300);
    if(randnum > 190)
    {
      BackwardAndTurnLeft();
      for(int16_t i=0;i<300;i++)
      {
        if(read_serial() == true)
        {
          break;
        }
        else
        {
          delay(2);
        }
      }
    }
    else
    {
      BackwardAndTurnRight();
      for(int i=0;i<300;i++)
      {
        if(read_serial() == true)
        {
          break;
        }
        else
        {
          delay(2);
        }
      }
    }
  }
  else
  {
    leftflag=false;
    rightflag=false;
    Forward();
  }
}

/**
 * \par Function
 *    IrProcess
 * \par Description
 *    The main function for IR control mode
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void IrProcess()
{
  if(ir == NULL)
  {
      ir = new MeInfraredReceiver(PORT_6);
      ir->begin();
  }
  ir->loop();
  irRead =  ir->getCode();
  if((irRead != IR_BUTTON_TEST) && (megapi_mode != IR_REMOTE_MODE))
  {
    return;
  }
  switch(irRead)
  {
    case IR_BUTTON_PLUS: 
      Forward();
      break;
    case IR_BUTTON_MINUS:
      Backward();
      break;
    case IR_BUTTON_NEXT:
      TurnRight();
      break;
    case IR_BUTTON_PREVIOUS:
      TurnLeft();
      break;
    case IR_BUTTON_9:
      ChangeSpeed(factor*9+minSpeed);
      break;
    case IR_BUTTON_8:
      ChangeSpeed(factor*8+minSpeed);
      break;
    case IR_BUTTON_7:
      ChangeSpeed(factor*7+minSpeed);
      break;
    case IR_BUTTON_6:
      ChangeSpeed(factor*6+minSpeed);
      break;
    case IR_BUTTON_5:
      ChangeSpeed(factor*5+minSpeed);
      break;
    case IR_BUTTON_4:
      ChangeSpeed(factor*4+minSpeed);
      break;
    case IR_BUTTON_3:
      ChangeSpeed(factor*3+minSpeed);
      break;
    case IR_BUTTON_2:
      ChangeSpeed(factor*2+minSpeed);
      break;
    case IR_BUTTON_1:
      ChangeSpeed(factor*1+minSpeed);
      break;
    case IR_BUTTON_0:
      Stop();
      break;
    case IR_BUTTON_TEST:
      Stop();
      while( ir->buttonState() != 0)
      {
        ir->loop();
      }
      megapi_mode = megapi_mode + 1;
      if(megapi_mode == MAX_MODE)
      { 
        megapi_mode = BLUETOOTH_MODE;
      }
      break;
    default:
      Stop();
      break;
  }
}

/**
 * \par Function
 *    line_model
 * \par Description
 *    The main function for Patrol Line navigation mode
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void line_model(void)
{
  uint8_t val;
  val = line.readSensors();
  moveSpeed=120;
  switch (val)
  {
    case S1_IN_S2_IN:
      Forward();
      LineFollowFlag=10;
      break;

    case S1_IN_S2_OUT:
       Forward();
      if (LineFollowFlag>1) LineFollowFlag--;
      break;

    case S1_OUT_S2_IN:
      Forward();
      if (LineFollowFlag<20) LineFollowFlag++;
      break;

    case S1_OUT_S2_OUT:
      if(LineFollowFlag==10) Backward();
      if(LineFollowFlag<10) TurnLeft1();
      if(LineFollowFlag>10) TurnRight1();
      break;
  }
}

/* Start our Firmware Code */
/**
 * \par Function
 *   translateNBlocks
 * \par Description
 *  The function used to move the car kit forward n blocks
 *  where each block is of MOVEMENT_BLOCK_SIZE_CM cm.
 * \param[in]
 *   nBlocks - The number of blocks to move
 * \par Output
 *  None
 * \return
 *  None
 * \par Others
 * None
 */
void translateNBlocks(uint8_t nBlocks){
  moveDistance(nBlocks * MOVEMENT_BLOCK_SIZE_CM);
}

/**
 * \par Function
 *    rotateByCase
 * \par Description
 *    The function used to rotate the car kit by a certain number of degrees
 *    depending on the received case number.
 * \param[in]
 *    caseNum - The number of degrees to rotate by
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void rotateByCase(uint8_t caseNum){
  switch(caseNum){
    case ROTATION_BY_90_DEGREES_COUNTERCLOCKWISE:
      TurnLeft90();
      break;
    case ROTATION_BY_90_DEGREES_CLOCKWISE:
      TurnRight90();
      break;
    case ROTATION_BY_180_DEGREES:
      TurnRight90();
      TurnRight90();
      break;
    default:
      break;
  }
}

/**
 * \par Function
 *    executePath
 * \par Description
 *    The function used to execute the path commands received from the phone app.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void executePath(void){
  isStart = false;
  uint8_t dataLen = readBuffer(2);
  uint8_t currIdx = 3;
  while ((currIdx-3) < dataLen)
  {
    uint8_t commandByte = readBuffer(currIdx);
    Serial.print("Command Byte: ");
    Serial.println(commandByte, HEX);
    uint8_t actionType = commandByte & 0x01;
    uint8_t actionValue = (commandByte >> 1) & 0x7F;
    currIdx++;

    Serial.print("Action Type: ");
    Serial.println(actionType, HEX);

    switch(actionType)
    {
      case TRANSLATION_CMD:
        {
          Serial.print("Translation by ");
          Serial.print(actionValue);
          Serial.println(" blocks");
          translateNBlocks(actionValue);
          // writeEnd();
          callOK();
        }
        break;
      case ROTATION_CMD:
        {
          Serial.print("Rotation case ");
          Serial.println(actionValue);
          rotateByCase(actionValue);
          callOK();
        }
        break;
      default:
        {
          Serial.println("Invalid Command");
        }
        break;
    }
    delay(500);
  }
}

/**
 * \par Function
 *    read_serial
 * \par Description
 *    The function used to process serial data.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Is there a valid command 
 * \par Others
 *    None
 */
boolean read_serial(void)
{
  boolean result = false;
  readSerial();
  while(isAvailable)
  {
    uint8_t currentByte = serialRead & 0xff;
    Serial.print("Received Byte: ");
    Serial.println(currentByte,HEX);
    result = true;
    if((currentByte == bluetoothHeadB) && (isStart == false))
    {
      if(prevc == bluetoothHeadA)
      {
        index=1;
        isStart = true;
      }
    }
    else
    {
      prevc = currentByte;
      if(isStart)
      {
        if(index == 2)
        {
          dataLen = currentByte; 
        }
        else if(index > 2)
        {
          dataLen--;
        }
        writeBuffer(index,currentByte);
      }
    }
    index++;
    if(index > 51)
    {
      index=0; 
      isStart=false;
    }
    readSerial();
  }

  if(isStart && (dataLen == 0) && (index > 3))
  { 
    isStart = false;
    Serial.println("Execute Path");
    executePath(); 
  }

  index = 0;
  return result;
}

/**
 * \par Function
 *    updatePetersSensors
 * \par Description
 *   The function used to update the sensors on the car kit and loop the encoders.
 * \param[in]
 *   None
 * \par Output
 *  None
 * \return
 * None
 * \par Others
 * None
 */
void updatePetersSensors(void){
  frontLeftEncoder->loop();
  rearRightEncoder->loop();
  if(millis() - update_sensor > 10) {
    update_sensor = millis();
    gyro_ext.fast_update();
  }
}

void moveFrontLeftEncoderForward(void){
  frontLeftEncoder->setMotorPwm(frontLeftEncoderSpeed);
}

void moveRearRightEncoderForward(void){
  rearRightEncoder->setMotorPwm(rearRightEncoderSpeed);
}

/**
 * \par Function
 *    Forward
 * \par Description
 *    This function use to control the car kit go forward.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Forward(void)
{
  moveFrontLeftEncoderForward();
  moveRearRightEncoderForward();
}

/**
 * \par Function
 *    Backward
 * \par Description
 *    This function use to control the car kit go backward.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Backward(void)
{
  frontLeftEncoder->setMotorPwm(moveSpeed);
  rearRightEncoder->setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    TurnLeft
 * \par Description
 *    This function use to control the car kit go backward and turn left.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnLeft(void)
{
  frontLeftEncoder->setMotorPwm(-frontLeftEncoderSpeed * 0.7);
  rearRightEncoder->setMotorPwm(rearRightEncoderSpeed * 0.7);
}

/**
 * \par Function
 *    getAnglePlusDegrees
 * \par Description
 *   This function use to calculate the target angle based on the current angle and the degrees to turn.
 * \param[in]
 *   angle - The current angle of the gyro
 * \param[in]
 *  degrees - The degrees to turn
 * \par Output
 *   None
 * \return
 *  The target angle
 * \par Others
 *  None
 */
double getAnglePlusDegrees(double angle, double degrees)
{
  double targetAngle = angle + degrees;
  if(targetAngle > 180)
  {
    targetAngle -= 360;
  }
  else if(targetAngle < -180)
  {
    targetAngle += 360;
  }
  return targetAngle;
}

/**
 * \par Function
 *    getAverageOfNZAngles
 * \par Description
 *   This function use to get the average of N angles along the z-axis.
 * \param[in]
 *   N - The number of angles to average
 * \par Output
 *   None
 * \return
 *  The average of the N angles
 * \par Others
 *  None
 */
double getAverageOfNZAngles(int N)
{
  double angle = 0;
  for(int i = 0; i < N; i++)
  {
    gyro_ext.update();
    angle += gyro_ext.getAngleZ();
    delay(100);
  }
  return angle / N;
}

/**
 * \par Function
 *    turnByDegrees
 * \par Description
 *  This function use is to control the car kit to turn a certain number of degrees.
 * \param[in]
 *  degrees - The number of degrees to turn
 * \par Output
 * None
 * \return
 * None
 * \par Others
 * None
 */
void turnByDegrees(double degrees){
  double angle = getAverageOfNZAngles(5);
  double targetAngle = getAnglePlusDegrees(angle, degrees);

  unsigned long startTime_ms = millis();
  unsigned long timeout_ms = 5000;
  int angleTolerance = 2;
  while((abs(targetAngle - angle) > angleTolerance) && ((millis() - startTime_ms) < timeout_ms))
  {
    if (isPathClear()) {
      if (degrees > 0)
      {
        TurnLeft();
      }
      else
      {
        TurnRight();
      }
    }
    else Stop();
    updatePetersSensors();
    angle = gyro_ext.getAngleZ();
    delay(10);
  }
  Stop();
}

/**
 * \par Function
 *    TurnLeft90
 * \par Description
 *    This function use is to control the car kit to turn left 90 degrees.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnLeft90(void)
{
  turnByDegrees(90);
}

/**
 * \par Function
 *    TurnRight90
 * \par Description
 *    This function use is to control the car kit to turn right 90 degrees.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnRight90(void)
{
  turnByDegrees(-90);
}

/**
 * \par Function
 *    TurnRight
 * \par Description
 *    This function use to control the car kit go backward and turn right.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnRight(void)
{
  frontLeftEncoder->setMotorPwm(frontLeftEncoderSpeed * 0.7);
  rearRightEncoder->setMotorPwm(-rearRightEncoderSpeed * 0.7);
}

/**
 * \par Function
 *    moveDistance
 * \par Description
 *   This function use to move the car kit a certain distance in centimeters.
 * \param[in]
 *  distance_cm - The distance to move in centimeters
 * \par Output
 *  None
 * \return
 * None
 * \par Others
 * None
 */
void moveDistance(float distance_cm){
  float pulses = distance_cm / distancePerPulse;
  moveForwardPulses(pulses);
}

void printEncoderValues(MeEncoderOnBoard *encoder, String name){
  Serial.print(name);
  Serial.print(" Cur Pos [deg]: ");
  Serial.println(encoder->getCurPos());
  Serial.print(name);
  Serial.print(" Pulse Pos: ");
  Serial.println(encoder->getPulsePos());
  Serial.print(name);
  Serial.print(" Cur PWM: ");
  Serial.println(encoder->getCurPwm());
  Serial.print(name);
  Serial.print(" Cur Speed [RPM]: ");
  Serial.println(encoder->getCurrentSpeed());
  Serial.println("-----------------------------");
  Serial.println("");
}  

/**
 * \par Function
 *    isPathClear
 * \par Description
 *    The function used to check if the path is clear by checking
 *    if there is a clearance of 20 cm using the ultrasonic sensor.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Is the path clear
 * \par Others
 *    None
 */
bool isPathClear(){
  return ultrasonic_sensor.distanceCm() > 20;
}

/**
 * \par Function
 *    moveForwardPulses
 * \par Description
 *   This function use to move the car kit a certain number of pulses.
 * \param[in]
 *  pulses - The number of pulses to move
 * \par Output
 *  None
 * \return
 * None
 * \par Others
 * None
 */
void moveForwardPulses(long pulses) {
  // Calculate the target pulse position
  long frontLeftTargetPulsePos = frontLeftEncoder->getPulsePos() - pulses;
  long rearRightTargetPulsePos = rearRightEncoder->getPulsePos() + pulses;

  // Set the motor speed
  Forward();

  // Define a tolerance for stopping
  const long tolerance = 10; // Adjust this value as needed
  double absLeftDiff = abs(frontLeftEncoder->getPulsePos() - frontLeftTargetPulsePos);
  double absRightDiff = abs(rearRightEncoder->getPulsePos() - rearRightTargetPulsePos);
  // Loop until the target position is reached within the tolerance
  while (absLeftDiff > tolerance && absRightDiff > tolerance) {
    if (isPathClear()) {
      // Update the motor speed
      if (absLeftDiff > tolerance) {
        moveFrontLeftEncoderForward();
        absLeftDiff = abs(frontLeftEncoder->getPulsePos() - frontLeftTargetPulsePos);
      }
      else frontLeftEncoder->setMotorPwm(0);

      if (absRightDiff > tolerance) {
        moveRearRightEncoderForward();
        absRightDiff = abs(rearRightEncoder->getPulsePos() - rearRightTargetPulsePos);
      }
      else rearRightEncoder->setMotorPwm(0);
    }
    else Stop();
    

    // Update the encoder state
    frontLeftEncoder->loop();
    rearRightEncoder->loop();

    // Add a small delay to prevent overwhelming the CPU
    delay(10);
    Serial.println("Stuck Here!");
  }

  // Stop the motor
  Stop();
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  while(!Serial){}
  while(!Serial2){}
  while(!Serial3){}
  delay(5);

  frontLeftEncoder->reset(SLOT1);
  rearRightEncoder->reset(SLOT2);
  attachInterrupt(frontLeftEncoder->getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(rearRightEncoder->getIntNum(), isr_process_encoder2, RISING);
  delay(5);

  gyro_ext.begin();
  delay(5);
  pinMode(13,OUTPUT);

  //Set Pwm 970Hz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);

  leftflag=false;
  rightflag=false;
  PID_angle.Setpoint = RELAX_ANGLE;
  PID_angle.P = 20;          // 20;
  PID_angle.I = 1;           // 1;
  PID_angle.D = 0.2;         // 0.2;
  PID_speed.P = 0.06;        // 0.06
  PID_speed.I = 0.005;       // 0.005
  PID_speed.D = 0;

  for(int i=0; i<2; i++)
  {
    encoders[i]->setPulse(8);
    encoders[i]->setRatio(46.67);
    encoders[i]->setPosPid(1.8, 0, 1.2);
    encoders[i]->setSpeedPid(PID_speed.P, PID_speed.I, PID_speed.D);
    encoders[i]->setMotionMode(DIRECT_MODE);
  }

  readEEPROM();

  Serial.print("Version: ");
  Serial.println(mVersion);
  update_sensor = lasttime_speed = lasttime_angle = millis();
  blink_time = millis();
}

/**
 * \par Function
 *    loop
 * \par Description
 *    main function for arduino
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void loop()
{
  if(millis() - blink_time > 1000)
  {
    blink_time = millis();
    blink_flag = !blink_flag;
    digitalWrite(13,blink_flag);
  }

  if(megapi_mode == BLUETOOTH_MODE)
  {
    read_serial();
  }
  else if(megapi_mode == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE)
  { 
    ultrCarProcess();
  }
  else if(megapi_mode == BALANCED_MODE)
  {
    gyro_ext.fast_update();
    balanced_model();
  }
  else if(megapi_mode == LINE_FOLLOW_MODE)
  {
    line_model();
  }
}
/* End our Firmware Code */