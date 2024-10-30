/*  Sketch that reads data from ACX1 controller via CAN bus
 *  and sends it via serial and wifi.
 *  https://sundays-ev.com/samurai/new/can-bus
 *  
 *  Adapter from MCP_CAN library: https://github.com/coryjfowler/MCP_CAN_lib
 */

/*
  * Wiring diagram ESP3286 <=> MCP2515 (Joy-it CAN module)
  * 3V3         - VCC
  * Vin (5V)    - VCC1
  * GND         - GND
  * D8 (GPIO15) - CS
  * D6          - SO
  * D7          - SI
  * D5          - SCK
  * D4 (GPIO2)  - INT
  */

#include <mcp_can.h>
#include <SPI.h>
#include <pubsubClient.h>
#include <Ticker.h>

#include <ESP8266WiFi.h>

// Serial TX Variables
unsigned long prevTX = 0;          // Variable to store last execution time
const unsigned int invlTX = 1000;  // One second interval constant

// CAN RX Variables
long unsigned int rxId;
long unsigned int rxId2;
long unsigned int rxId3;
long unsigned int rxId4;
long unsigned int rxId5;
long unsigned int rxId6;
long unsigned int rxId7;
long unsigned int rxId8;
unsigned char len;
byte rxBuf[8];
byte rxBuf2[8];
byte rxBuf3[8];
byte rxBuf4[8];
byte rxBuf5[8];
byte rxBuf6[8];
byte rxBuf7[8];
byte rxBuf8[8];
byte tpoBuf[8];
byte tpoBuf2[8];
byte tpoBuf3[8];
byte tpoBuf4[8];
byte tpoBuf5[8];
byte tpoBuf6[8];
byte tpoBuf7[8];
byte tpoBuf8[8];

// ACX1 TPOs
double temp = 0;
double range = 0;
double soc = 0;
double amps = 0;
double key = 0;
double odo = 0;
double BattA = 0;
double BattV = 0;
double BattW = 0;
double QCamps = 0;
double CDCV = 0;
double CDCA = 0;
double CACV = 0;
double CACA = 0;
float soclevel;
float socleveltemp;
float rangekm;
float templevel;
float amplevel;
float keystate;
float odometer;
float Bampers;
float Bvoltage;
float Bwatts;
float Chademoamps;
float ChargeDCV;
float ChargeDCA;
float ChargeACV;
float ChargeACA;

// Serial Output String Buffer
char msgString[128];

// CAN0 INT and CS
#define CAN0_INT D2  // INT in pin 2
MCP_CAN CAN0(D8);    // CS in pin 15

// Webserver related
WiFiServer server(80);
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

#define MQTT_HOST "hairdresser.cloudmqtt.com"
#define MQTT_PORT 15745
#define MQTT_PUB_TEMP "tele/CAN/soc"
#define MQTT_PUB_TEMP2 "tele/CAN/range"
#define MQTT_PUB_TEMP3 "tele/CAN/temp"
#define MQTT_PUB_TEMP4 "tele/CAN/amps"
#define MQTT_PUB_TEMP5 "tele/CAN/key"
#define MQTT_PUB_TEMP6 "tele/CAN/odo"
#define MQTT_PUB_TEMP7 "tele/CAN/batta"
#define MQTT_PUB_TEMP8 "tele/CAN/battv"
#define MQTT_PUB_TEMP9 "tele/CAN/battw"
#define MQTT_PUB_TEMP10 "tele/CAN/qcamps"
#define MQTT_PUB_TEMP11 "tele/CAN/ChargeDCV"
#define MQTT_PUB_TEMP12 "tele/CAN/ChargeDCA"
#define MQTT_PUB_TEMP13 "tele/CAN/ChargeACV"
#define MQTT_PUB_TEMP14 "tele/CAN/ChargeACA"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10000;       // Interval at which to publish sensor readings

void setup() {
  Serial.begin(115200);
  Serial.println();

  startWifi();
  startCan();

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("uqctrigg", "GqofEFGIW-Rn");
  connectToMqtt();

  Serial.println("Setup done.");
}

void loop() {

  if (!digitalRead(CAN0_INT)) {  // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);    //SOC
    CAN0.readMsgBuf(&rxId2, &len, rxBuf2);  //temp
    CAN0.readMsgBuf(&rxId3, &len, rxBuf3);  //range
    CAN0.readMsgBuf(&rxId4, &len, rxBuf4);  //heater amps
    CAN0.readMsgBuf(&rxId5, &len, rxBuf5);  //key - odo
    CAN0.readMsgBuf(&rxId6, &len, rxBuf6);  //QCamps
    CAN0.readMsgBuf(&rxId7, &len, rxBuf7);  //Batt A V W
    CAN0.readMsgBuf(&rxId8, &len, rxBuf8);  //charge A V W


    // if TPO, read data
    if (rxId == 0x374) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf[i] = rxBuf[i];
      }

      soc = (rxBuf[0] - 10) / 2.0;
      soclevel = soc;
      //((float *)rxBuf)[7] = soclevel;
      //soclevel = (socleveltemp - 10) / 2.0;
    }
    if (rxId2 == 0x286) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf2[i] = rxBuf2[i];
      }

      temp = rxBuf2[3] - 50.0;
      templevel = temp;
    }
    if (rxId3 == 0x346) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf3[i] = rxBuf3[i];
      }

      range = rxBuf3[7];
      rangekm = range;
    }
    if (rxId4 == 0x384) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf4[i] = rxBuf4[i];
      }

      amps = rxBuf4[4] / 10.0;
      amplevel = amps;
    }
    if (rxId5 == 0x412) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf5[i] = rxBuf5[i];
      }

      key = rxBuf5[0];
      odo = ((rxBuf5[2] * 256 * 256) + (rxBuf5[3] * 256) + rxBuf5[4]);
      odometer = odo;
      keystate = key;
    }
    if (rxId6 == 0x697) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf6[i] = rxBuf6[i];
      }

      QCamps = rxBuf6[2];
      Chademoamps = QCamps;
    }
    if (rxId7 == 0x373) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf7[i] = rxBuf7[i];
      }

      BattV = (((rxBuf7[4] * 256) + rxBuf7[5]) / 10);
      BattA = (((rxBuf7[2] * 256) + rxBuf7[3] - 32768) / 100) + 0.66;
      BattW = BattA * BattV;
      Bampers = BattA;
      Bvoltage = BattV;
      Bwatts = BattW;
    }
    if (rxId8 == 0x389) {

      // debug:
      for (byte i = 0; i < 7; i++) {
        tpoBuf8[i] = rxBuf8[i];
      }
      CDCV = 2 * (rxBuf8[0] + 0.5);
      CACV = rxBuf8[1];
      CDCA = rxBuf8[2] / 10.0;
      CACA = rxBuf8[6] / 10.0;
      CDCV = ChargeDCV;
      CDCA = ChargeDCA;
      CACV = ChargeACV;
      CACA = ChargeACA;

    }
  }


  /*// send via serial
  if(millis() - prevTX >= invlTX) {                    // Send this at a one second interval. 
    prevTX = millis();

    /*
    for(byte i = 0; i<7; i++){
      sprintf(msgString, " 0x%.2X", tpoBuf[i]);
      Serial.print(msgString);
    }
    Serial.println();
    
      
    sprintf(msgString, "temp=%d range=%d%km amps=%dC SoC=%d%%", temp, range, amps, soc);  
    Serial.println(msgString);
  }
  */
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New temperature readings
    // Publish an MQTT message on topic esp/ds18b20/temperature
    if ((rangekm > 0) && (rangekm < 255)) {
      mqttClient.publish(MQTT_PUB_TEMP2, 1, true, String(rangekm, 0).c_str());
    }
    if (soclevel > 0) {
      mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(soclevel, 0).c_str());
    }
      mqttClient.publish(MQTT_PUB_TEMP4, 1, true, String(amplevel, 0).c_str());
    
    if (templevel != 0) {
      mqttClient.publish(MQTT_PUB_TEMP3, 1, true, String(templevel, 0).c_str());
    }
    if (keystate == 254) {
      mqttClient.publish(MQTT_PUB_TEMP5, 1, true, "Ready");
    } else {
      mqttClient.publish(MQTT_PUB_TEMP5, 1, true, "Off");
    }
    if (odometer != 0) {
      mqttClient.publish(MQTT_PUB_TEMP6, 1, true, String(odometer, 0).c_str());
    }
    mqttClient.publish(MQTT_PUB_TEMP7, 1, true, String(Bampers, 0).c_str());
    mqttClient.publish(MQTT_PUB_TEMP8, 1, true, String(Bvoltage, 0).c_str());
    mqttClient.publish(MQTT_PUB_TEMP9, 1, true, String(Bwatts, 0).c_str());
    mqttClient.publish(MQTT_PUB_TEMP10, 1, true, String(Chademoamps, 0).c_str()); 
    mqttClient.publish(MQTT_PUB_TEMP11, 1, true, String(ChargeDCV, 0).c_str());
    mqttClient.publish(MQTT_PUB_TEMP12, 1, true, String(ChargeDCA, 0).c_str());
    mqttClient.publish(MQTT_PUB_TEMP13, 1, true, String(ChargeACV, 0).c_str());
    mqttClient.publish(MQTT_PUB_TEMP14, 1, true, String(ChargeACA, 0).c_str());

  }

  //handleWeb();
}

void startWifi() {
  WiFi.begin("HunorCar", "Jucymucy8581");
  Serial.print("Connecting to wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  //server.begin();
  //Serial.println("Webserver started.");
}

void startCan() {
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}

/*String getHtmlPage() {
  sprintf(msgString, "temp=%d range=%d%% amps=%dC SoC=%d%%", temp, range, amps, soc);
  String htmlPage =
     String("HTTP/1.1 200 OK\r\n") +
            "Content-Type: text/html\r\n" +
            "Connection: close\r\n" +  // the connection will be closed after completion of the response
            // "Refresh: 5\r\n" +  // refresh the page automatically every 5 sec
            "\r\n" +
            "<!DOCTYPE HTML>" +
            "<html>" + msgString + "</html>" +
            "\r\n";
  return htmlPage;
}

void handleWeb() {
  WiFiClient client = server.available();
  // wait for a client (web browser) to connect
  if (client) {
    Serial.println("\n[Client connected]");
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
       currentTime = millis(); 
      // read line by line what the client (web browser) is requesting
      if (client.available())
      {
        String line = client.readStringUntil('\r');
        Serial.print(line);
        // wait for end of client's request, that is marked with an empty line
        if (line.length() == 1 && line[0] == '\n')
        {
          client.println(getHtmlPage());
          break;
        }
      }
    }
    delay(10); // give the web browser time to receive the data

    // close the connection:
    client.stop();
    Serial.println("[Client disonnected]");
  }
}
*/
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
