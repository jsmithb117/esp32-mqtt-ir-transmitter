#include <queue>

// IR Receiver module
const uint16_t irLedPin = 5;

// IR transmitter module
const uint16_t kRecvPin = 4;
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 15;



// BMP280/AHT20 constants
#define SEALEVELPRESSURE_HPA (1013.25)

const char *DEVICE_TOPIC = "home/living/entertainment/tv/controller";
// Remote control commands and topic assignments
// Use the or IRrecvDumpV3 example to find the codes for your remote
#define PWR 0x2FD48B7
const char *PWR_TOPIC = "home/living/entertainment/tv/controller/PWR";
#define CH_PLUS 0x2FDD827
const char *CH_PLUS_TOPIC = "home/living/entertainment/tv/controller/CH_PLUS";
#define CH_MINUS 0x2FDF807
const char *CH_MINUS_TOPIC = "home/living/entertainment/tv/controller/CH_MINUS";
#define V_PLUS 0x2FD58A7
const char *V_PLUS_TOPIC = "home/living/entertainment/tv/controller/V_PLUS";
#define V_MINUS 0x2FD7887
const char *V_MINUS_TOPIC = "home/living/entertainment/tv/controller/V_MINUS";
#define MUTE 0x2FD08F7
const char *MUTE_TOPIC = "home/living/entertainment/tv/controller/MUTE";
#define SLP 0x2FDA857
const char *SLP_TOPIC = "home/living/entertainment/tv/controller/SLP";
#define INPUT 0x2FDF00F
const char *INPUT_TOPIC = "home/living/entertainment/tv/controller/INPUT";
#define DEC 0x2FD50AF
const char *DEC_TOPIC = "home/living/entertainment/tv/controller/DEC";
#define ONE 0x2FD807F
const char *ONE_TOPIC = "home/living/entertainment/tv/controller/ONE";
#define TWO 0x2FD40BF
const char *TWO_TOPIC = "home/living/entertainment/tv/controller/TWO";
#define THREE 0x2FDC03F
const char *THREE_TOPIC = "home/living/entertainment/tv/controller/THREE";
#define FOUR 0x2FD20DF
const char *FOUR_TOPIC = "home/living/entertainment/tv/controller/FOUR";
#define FIVE 0x2FDA05F
const char *FIVE_TOPIC = "home/living/entertainment/tv/controller/FIVE";
#define SIX 0x2FD609F
const char *SIX_TOPIC = "home/living/entertainment/tv/controller/SIX";
#define SEVEN 0x2FDE01F
const char *SEVEN_TOPIC = "home/living/entertainment/tv/controller/SEVEN";
#define EIGHT 0x2FD10EF
const char *EIGHT_TOPIC = "home/living/entertainment/tv/controller/EIGHT";
#define NINE 0x2FD906F
const char *NINE_TOPIC = "home/living/entertainment/tv/controller/NINE";
#define ZERO 0x2FD00FF
const char *ZERO_TOPIC = "home/living/entertainment/tv/controller/ZERO";
#define ENTER 0x2FD916E
const char *ENTER_TOPIC = "home/living/entertainment/tv/controller/ENTER";
#define PREV_CH 0x2FDE817
const char *PREV_CH_TOPIC = "home/living/entertainment/tv/controller/PREV_CH";

// map topics to IR codes
struct TopicIRCode {
  const char* topic;
  uint32_t irCode;
};
TopicIRCode topicToIRCode[] = {
  {PWR_TOPIC, PWR},
  {CH_PLUS_TOPIC, CH_PLUS},
  {CH_MINUS_TOPIC, CH_MINUS},
  {V_PLUS_TOPIC, V_PLUS},
  {V_MINUS_TOPIC, V_MINUS},
  {MUTE_TOPIC, MUTE},
  {SLP_TOPIC, SLP},
  {INPUT_TOPIC, INPUT},
  {DEC_TOPIC, DEC},
  {ONE_TOPIC, ONE},
  {TWO_TOPIC, TWO},
  {THREE_TOPIC, THREE},
  {FOUR_TOPIC, FOUR},
  {FIVE_TOPIC, FIVE},
  {SIX_TOPIC, SIX},
  {SEVEN_TOPIC, SEVEN},
  {EIGHT_TOPIC, EIGHT},
  {NINE_TOPIC, NINE},
  {ZERO_TOPIC, ZERO},
  {ENTER_TOPIC, ENTER},
  {PREV_CH_TOPIC, PREV_CH}
};

const char* subscribeTopics[] = {
  PWR_TOPIC,
  CH_PLUS_TOPIC,
  CH_MINUS_TOPIC,
  V_PLUS_TOPIC,
  V_MINUS_TOPIC,
  MUTE_TOPIC,
  SLP_TOPIC,
  INPUT_TOPIC,
  DEC_TOPIC,
  ONE_TOPIC,
  TWO_TOPIC,
  THREE_TOPIC,
  FOUR_TOPIC,
  FIVE_TOPIC,
  SIX_TOPIC,
  SEVEN_TOPIC,
  EIGHT_TOPIC,
  NINE_TOPIC,
  ZERO_TOPIC,
  ENTER_TOPIC,
  PREV_CH_TOPIC
};


// MQTT constants
const String MQTT_CLIENT_ID = "esp-thermostat-";
IPAddress mqttServer(192, 168, 1, 41);

// Topics
//$building/$room/$domain/$device_name/$topic_class/$data_type/$unit(optional)
const char *PRESSURE_TOPIC = "home/living/entertainment/tv/controller/pressure/pascal";
const char *TEMPERATURE_TOPIC = "home/living/entertainment/tv/controller/temperature/fahrenheit";
const char *HUMIDITY_TOPIC = "home/living/entertainment/tv/controller/humidity/percent";
const char *LWT = "home/living/entertainment/tv/controller/logged-in/boolean";
const char *IR_TOPIC = "home/living/entertainment/tv/controller/ircode";
const char *RSSI_TOPIC = "home/living/entertainment/tv/controller/rssi";

// create a queue of MQTT messages to send
// Define the MQTTMessage struct
struct MQTTMessage {
  String topic;
  String payload;
};

// Declare the queue of MQTT messages
std::queue<MQTTMessage> mqttMessageQueue;