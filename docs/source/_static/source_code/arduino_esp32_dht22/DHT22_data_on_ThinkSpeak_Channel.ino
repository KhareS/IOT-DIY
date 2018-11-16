
/* Publish DHT22 sensor data to ThingSpeak server

 * This sketch connects Esp32 NodeMCU to WiFi network, 
 * reads temperature and humidity data from DTH22 sensor, 
 * publishes these readings to locally running mosquitto server at Raspberry Pi 3, 
 * and also at ThingSpeak public channel. LED at pin 32 is used for test/debug purpose only.
 * 
 * ThingSpeak public channel: https://thingspeak.com/channels/470442

   The circuit:
  * pin 16 connected to DHT22 sensor

   created 09 Apr 2018
   by Sharad Khare 
 */


/*
 * Sketch uses DHTesp library, Adafruit Unified Sensor Library & DHT Sensor Library  
 * sometimes misses the reading on my ESP32 Node MCU board
 * Download DHTesp library from: https://github.com/beegee-tokyo/DHTesp/ 
 */

#include <WiFi.h>                               // WiFi network
#include <PubSubClient.h>                       // Publishing payload on MQTT server

#include "DHTesp.h"                             // download library from https://github.com/beegee-tokyo/DHTesp
#include "ESP32Ticker.h"                        // Ticker library has been merged into the Arduino core for ESP32

int ledPin = 32;                                // For testing/debugging

/*
* WiFi connection
*/
const char* ssid = "enter_your_SSID_here";      // WiFi SSID
const char* pwd = "enter_your_Password_here";   // WiFi Password


/*
 * Number of attempts
 */
byte noOfAttemptForWiFiConn = 30;               // Number of attempts, give user to try for WiFi & MQTT server connection.
                                                // 120 = 1 minute. Restricting infinite attempts


/*
 * MQTT Server
 */
const char* MQTTServer = "192.168.43.92";                   // Yours may be different
const int MQTTPort = 1883;
const char* MQTTUser = "";
const char* MQTTPwd = "";
const char* Mqqt_getTemperature = "home/room2/temperature"; // Topic
const char* Mqqt_getHumidity = "home/room2/humidity";       // Topic
const char* clientID = "ESP32Client";                       // The client id identifies the ESP32 device. 
                                                            // Think of it a bit like a hostname (Or just a name, like Sharad).
const char* thsclientID = "ThinkSpeakClient";               // The client id identifies the ESP32 device. 
                                                            

/*
 * ThinkSpeak Server settings:
 * char* topic="channels/<channel_ID/publish/<channelAPI>
 * Channel_ID - Your channel Id
 * Channel API - Your API Key
 */
char* thinkSpeakTopic = "channels/enter_your_channel_id_here/publish/enter_your_API_key_here";
char* thinkSpeakServer = "mqtt.thingspeak.com";


/*
 *  Object of class WiFiClient allows to establish a connection to a defined IP and port. 
 *  Nevertheless, we will not explicitly use this object because it will be used by the MQTT library under the hood.
 */
WiFiClient espClient;
WiFiClient thsWiFiClient;
//PubSubClient psClient(espClient);
PubSubClient psClient(MQTTServer, MQTTPort, espClient);
PubSubClient thsClient(thinkSpeakServer, MQTTPort, thsWiFiClient);


/*
 * DHT22 Sensor 
 */
#define DHTPIN 16                               // Pin connected to the DHT sensor.
#define DHTTYPE           DHT22                 // DHT 22 (AM2302)

DHTesp dhtEsp22;

/*
 * Function definition
 * 
 */
void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

/*
 * Initialization
 * 
 */
/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;




/*
 * function to connecting a selected ssid network
 */
void connect_WiFi_Network(){

  // Conneting a WiFi access point
  WiFi.begin(ssid, pwd);

  byte tryConn = 0;

  Serial.println("------------------------------------------");

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(". ");                         // Every dot represent a number of attempts
    /*
     *  Can't go forever
     *  check here how many number of attempts user has made
     */
     tryConn++;
     if(tryConn > noOfAttemptForWiFiConn){
      break;
     }
  }

    /*
     // Debugging purpose
    Serial.print("\nTryConn: ");
    Serial.println(tryConn);
    Serial.print("noOfAttemptForWiFiConn: ");
    Serial.println(noOfAttemptForWiFiConn);
    */

 /*   
  *    For debugging purpose to which WiFi network board is connected
  *    Comment in production
 */
 if(WiFi.status() == WL_CONNECTED){
    Serial.println("");
    Serial.println("------------------------------------------");
    Serial.print("Connected to WiFi network: ");
    Serial.println(ssid);
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress()); 
    Serial.print("Encryption Type: ");
    //String translateEncrytionType = translateEncryptionType(WiFi.encryptionType(0));
    Serial.println(WiFi.encryptionType(0));
    Serial.println("------------------------------------------");
    delay(10);
 } 
  

  /*
   * Show user a message if not connected
   */
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("------------------------------------------");
    Serial.print("WiFi not connected!!! You have exhausted all of your (");
    Serial.print(noOfAttemptForWiFiConn);
    Serial.println(") attempts for connecting to a WiFi network.");
    Serial.println("------------------------------------------");
  }
}

/*
 * Disconnect to a connected WiFi network
 */
void disconnect_network(){
    WiFi.disconnect(true);            // Pass "true" to disconnect from WiFi network.
    Serial.println("Disconnected.");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
}


/**
 * initTemp
 * Setup DHT library
 * Setup task and timer for repeated measurement
 * @return bool
 *    true if task and timer are started
 *    false if task or timer couldn't be started
 */
bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dhtEsp22.setup(DHTPIN, DHTesp::DHT22);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
      tempTask,                       /* Function to implement the task */
      "tempTask ",                    /* Name of the task */
      4000,                           /* Stack size in words */
      NULL,                           /* Task input parameter */
      5,                              /* Priority of the task */
      &tempTaskHandle,                /* Task handle. */
      1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
       Serial.println("Environment data update in every (20) seconds...");
    // Start update of environment data every 20 seconds
    //   tempTicker.attach(20, triggerGetTemp);
    // Start update of environment data every 60 seconds
       tempTicker.attach(60, triggerGetTemp);
       Serial.println("");
  }
  return true;
}

/**
 * triggerGetTemp
 * Sets flag dhtUpdated to true for handling in loop()
 * called by Ticker getTempTimer
 */
void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
     xTaskResumeFromISR(tempTaskHandle);
  }
}


/**
 * Task to reads temperature from DHT22 sensor
 * @param pvParameters
 *    pointer to task parameters
 */
void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}


/**
 * getTemperature
 * Reads temperature from DHT22 sensor
 * @return bool
 *    true if temperature could be acquired
 *    false if acquisition failed
*/
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dhtEsp22.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dhtEsp22.getStatus() != 0) {
    Serial.println("DHT22 error status: " + String(dhtEsp22.getStatusString()));
    return false;
  }

  float heatIndex = dhtEsp22.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dhtEsp22.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dhtEsp22.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch(cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  // Show all readings on serial port
  Serial.println(" Temp.:" + String(newValues.temperature) + "*C; Humid.:" + String(newValues.humidity) + "%; Heat Index:" + String(heatIndex) + "; Dew Point:" + String(dewPoint) + "; " + comfortStatus);

  char tmpMsgBuffer[20];                     // make sure this is big enough to hold your string
  char hmdMsgBuffer[20];                     // make sure this is big enough to hold your string
  char* pointer_to_temp_string;
  char* pointer_to_humid_string;

  pointer_to_temp_string = dtostrf(newValues.temperature, 6, 2, tmpMsgBuffer);
  pointer_to_humid_string = dtostrf(newValues.humidity, 6, 2, hmdMsgBuffer);

  /*
  //Debugging purpose - reading get converted into String correctly
  Serial.println(pointer_to_temp_string);
  Serial.println(pointer_to_humid_string);
  Serial.println("----");    
  */

    // Publish sensor reading on MQTT broker
  
    // Temperature
    if (psClient.publish(Mqqt_getTemperature, pointer_to_temp_string)){
      Serial.println("Temperature reading published!");
    } else {
      Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
      psClient.connect(clientID);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      psClient.publish(Mqqt_getTemperature, pointer_to_temp_string);    
    }
  
    // Humidity
    if (psClient.publish(Mqqt_getHumidity, pointer_to_humid_string)){
      Serial.println("Humidity reading published!");
    } else {
      Serial.println("Humidity failed to send. Reconnecting to MQTT Broker and trying again");
      psClient.connect(clientID);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      psClient.publish(Mqqt_getHumidity, pointer_to_humid_string);    
    }

   // Publish sensor reading on ThinkSpeak Server

    String payload="field1=";
    payload += pointer_to_temp_string;
    payload += "&field2=";
    payload += pointer_to_humid_string;
    payload += "&status=MQTTPUBLISH";

    Serial.print("Sending payload: ");
    Serial.println(payload);

    if (!thsClient.connected()){
      thsClient.connect(thsclientID);
    }      

    if (thsClient.connected()){
        // publishing payload in one go
        if (thsClient.publish(thinkSpeakTopic, (char*) payload.c_str())){
          Serial.println("Payload published!");
        } else {
          Serial.println("Publish failed to send on ThinkSpeak server. Reconnecting to MQTT Broker and trying again");
        }
    } else {
      Serial.println("\nConnection to ThinkSpeak MQTT Broker failed...");
      Serial.print("failed with state: ");
      Serial.print(thsClient.state());
      Serial.println("\n------------------------------------------");
    } 
   
  
      Serial.println("\n------------------------------------------");
  
  return true;
}



void connect_ThinkSpeak_Server(){
  
  //psClient.setServer(MQTTServer, MQTTPort);

  byte tryConn = 0;

  Serial.println("------------------------------------");
  Serial.println("Connecting to ThinkSpeak broker...");


    // Connect to MQTT Broker
   while (!thsClient.connected()) {
        delay(20);
        Serial.print(". ");
  
       //  Can't go forever
       //  check here how many number of attempts user has made
       tryConn++;
       if(tryConn > noOfAttemptForWiFiConn){
        break;
       }
      
    } //end while

    // Connected
    if (thsClient.connect(thsclientID)) {
      Serial.println("\nThinkSeak MQTT server connected!");
    } else {
     // Show reason of not connecting to user 
      Serial.println("\nConnection to ThinkSpeak MQTT Broker failed...");
      Serial.print("failed with state ");
      Serial.print(thsClient.state());
      Serial.println("\n------------------------------------------");
    }
}


void connect_MQTT_Server(){
  
  //psClient.setServer(MQTTServer, MQTTPort);

  byte tryConn = 0;

  Serial.println("------------------------------------");
  Serial.println("Connecting to MQTT broker...");


    // Connect to MQTT Broker
   while (!psClient.connected()) {
        delay(20);
        Serial.print(". ");
  
       //  Can't go forever
       //  check here how many number of attempts user has made
       tryConn++;
       if(tryConn > noOfAttemptForWiFiConn){
        break;
       }
      
    } //end while

    // Connected
    if (psClient.connect(clientID)) {
      Serial.println("\nMQTT broker connected!");
    } else {
     // Show reason of not connecting to user 
      Serial.println("\nConnection to MQTT Broker failed...");
      Serial.print("failed with state ");
      Serial.print(psClient.state());
      Serial.println("\n------------------------------------------");
    }
}





void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  
  pinMode(ledPin, OUTPUT);                      // Test LED

  pinMode(DHTPIN, INPUT);                       // DHT22

  /*    
   *     Required in setup before connecting a WiFi network as station
   */
  // Set WiFi to station mode and disconnect from an Access Point (AP); if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  connect_WiFi_Network();                       // Re-connect to a WiFi network

  
  Serial.println("DHT ESP32 example with tasks");
  initTemp();

  connect_MQTT_Server();                        // Connect to MQTT Server

  connect_ThinkSpeak_Server();                  // Connect to MQTT Server

  Serial.print("Topic for local MQTT servers: ");
  Serial.println(Mqqt_getTemperature);
  Serial.println(Mqqt_getHumidity);

  Serial.print("Topic for ThinkSpeak servers: ");
  Serial.println(thinkSpeakTopic);

  Serial.println("\nSetup done"); 

  // Disconnect from WiFi network when done
  // disconnect_network();

}

void loop() {

  digitalWrite(ledPin, HIGH);

  if (!tasksEnabled) {
    // Wait 2 seconds to let system settle down
    delay(2000);
    // Enable task that will read values from the DHT sensor
    tasksEnabled = true;
    if (tempTaskHandle != NULL) {
      vTaskResume(tempTaskHandle);
    }
  } 
  

  digitalWrite(ledPin, LOW);
  yield();

}
