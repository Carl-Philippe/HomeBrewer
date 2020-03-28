#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#ifndef STASSID
#define STASSID "SmartRGE1BB"
#define STAPSK  "s9003502"
#endif

#define DEBUG false //Remove to activate serial prints 
#define SENSOR_PIN D4
#define SSR_PIN D6

// Change the credentials below, so your ESP8266 connects to your router
const char* ssid = STASSID;
const char* password = STAPSK;

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char* mqtt_server = "192.168.1.12";
// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
WiFiClient espClient;
PubSubClient client(espClient);

OneWire oneWire(SENSOR_PIN); //Setup the onewire instance to communicate with all OneWire devices
DallasTemperature sensors(&oneWire); // Pass the OneWire reference to DallasTemperature

bool node_red_connection = true;
bool cooling = false;

//PID settings
double temp_now, Output, Setpoint;
double Kp = 250, Ki = 10, Kd = 0;

PID coolingPID(&temp_now, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);  //PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
int WindowSize = 20000;
unsigned long windowStartTime;

// Timers auxiliar variables
long now = millis();
long lastMeasure = 0;

// Don't change the function below. This functions connects your ESP8266 to your router
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();

    Serial.println("");
    Serial.print("WiFi connected - ESP IP address: ");
    Serial.println(WiFi.localIP());
    
    ArduinoOTA.setHostname("Homebrewer"); // Identifiants du OTA
    ArduinoOTA.setPassword("Carlphilippe2");
    
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
    ArduinoOTA.begin();
    ArduinoOTA.setHostname("Homebrewer"); // Identifiants du OTA
    ArduinoOTA.setPassword("Carlphilippe2");
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that
// your ESP8266 is subscribed you can actually do something
void callback(String topic, byte* message, unsigned int length) {
  if(DEBUG){
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
  }
  String messageTemp;

  for (int i = 0; i < length; i++) {
    if(DEBUG){Serial.print((char)message[i]);}
    messageTemp += (char)message[i];
  }
  if(DEBUG){Serial.println();}

  // Feel free to add more if statements to control more GPIOs with MQTT

  if (topic == "homebrewer/connection") {
    if (messageTemp == "test") {
      if(DEBUG){Serial.println("Connection test from node-red");}
      if (node_red_connection)
        client.publish("homebrewer/connection", "true");
      else
        client.publish("homebrewer/connection", "false");
    }
    else if (messageTemp == "true") {
      node_red_connection = true;
      client.publish("homebrewer/connection", "true");
    }
    else if (messageTemp == "disconnect") {
      if(DEBUG){Serial.println("Closed connection, waiting for new one");}
      node_red_connection = false;
      client.publish("homebrewer/connection", "false");
    }
  }
  // If a message is received on the topic room/lamp, you check if the message is either on or off. Turns the lamp GPIO according to the message
  if (node_red_connection) {
    ////////// PLACE CALLBACKS HERE ////////////////
    if (topic == "homebrewer/temp_control") {
      if (messageTemp == "true") {
        cooling = true;
      }
      else if (messageTemp == "false") {
        cooling = false;
      }
      /*Serial.print("Changing Peltier State to ");
        if(messageTemp == "on"){
        digitalWrite(SSR_PIN, HIGH);
        }
        else if(messageTemp == "off"){
        digitalWrite(SSR_PIN, LOW);
        }
        Serial.print("messageTemp: ");Serial.println(messageTemp);*/
    }
    if (topic == "homebrewer/Setpoint") {
      Setpoint = atof((char*)message);
      if(DEBUG){Serial.print("Setpoint: ");Serial.println(Setpoint);}
    }
  }
}

// This functions reconnects your ESP8266 to your MQTT broker
// Change the function below if you want to subscribe to more topics with your ESP8266
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if(DEBUG){Serial.print("Attempting MQTT connection...");}
    // Attempt to connect
    /*
      YOU MIGHT NEED TO CHANGE THIS LINE, IF YOU'RE HAVING PROBLEMS WITH MQTT MULTIPLE CONNECTIONS
      To change the ESP device ID, you will have to give a new name to the ESP8266.
      Here's how it looks:
       if (client.connect("ESP8266Client")) {
      You can do it like this:
       if (client.connect("ESP1_Office")) {
      Then, for the other ESP:
       if (client.connect("ESP2_Garage")) {
      That should solve your MQTT multiple connections problem
    */
    if (client.connect("ESP8266Client")) {
      if(DEBUG){Serial.println("connected");}
      // -------------------------------------------------------------------------------------------------------------------------------- Subscribe or resubscribe to a topic
      client.subscribe("homebrewer/temp_control");
      client.subscribe("homebrewer/Setpoint");
      client.subscribe("homebrewer/connection");
    } else {
      if(DEBUG){Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");}
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void temp_control() {
  if (now - lastMeasure > 5000) {
      lastMeasure = now;
      coolingPID.Compute();   //  turn the output pin on/off based on pid output
  
  if(DEBUG){Serial.print("Output: ");Serial.println(Output);}
  now = millis();
  if (now - windowStartTime > WindowSize)
  {
    windowStartTime = millis(); //time to shift the Relay Window
  }
  else if (Output > now - windowStartTime) digitalWrite(SSR_PIN, HIGH);
  else digitalWrite(SSR_PIN, LOW);
  }
}

void temp_acquisition() {
  // Send the command to get temperatures.
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // Check if any reads failed and exit early (to try again).
  if (isnan(tempC) || tempC <= -10) {
    if(DEBUG){Serial.println("Failed to read from Dallas sensor!");}
    return;
  }
  else {
    char temperature[10];
    dtostrf(tempC, 10, 2, temperature);/*char *dtostrf(double val, signed char width, unsigned char prec, char *s)
                                              val : Variable décimale à convertir
                                              width : Taille de la chaîne cible (le caractère . doit être compté)
                                              prec : Nombre de chiffres après la virgule
                                              s : Tableau contenant la chaîne de caratères*/
    if (node_red_connection) {
      // Publishes Temperature value
      client.publish("homebrewer/temperature", temperature);
      //client.publish("room/humidity", humidityTemp);
    }
    if(DEBUG){Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" *C ");}
    temp_now = (double)tempC;
  }
}
  // The setup function sets your ESP GPIOs to Outputs, starts the serial communication at a baud rate of 115200
  // Sets your mqtt broker and sets the callback function
  // The callback function is what receives messages and actually controls the LEDs
  void setup() {
    pinMode(SSR_PIN, OUTPUT);
    digitalWrite(SSR_PIN,HIGH);
    sensors.begin();

    if(DEBUG){Serial.begin(115200);}
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    temp_now = 0; // input
    Setpoint = 0; // setpoint
    coolingPID.SetOutputLimits(0, WindowSize);   //tell the PID to range between 0 and the full window size
    coolingPID.SetMode(AUTOMATIC);  //turn the PID on
    coolingPID.SetSampleTime(5000);
  }

  // For this project, you don't need to change anything in the loop function. Basically it ensures that you ESP is connected to your broker
  void loop() {

    if (!client.connected())
      reconnect();
    if (!client.loop())
      client.connect("ESP8266Client");
    now = millis();
    // Publishes new temperature every 5 seconds
    if (now - lastMeasure > 5000) {
      lastMeasure = now;
      temp_acquisition();
    }
    if (cooling)
            temp_control();
    else
      digitalWrite(SSR_PIN,LOW);    

    ArduinoOTA.handle();
  }
