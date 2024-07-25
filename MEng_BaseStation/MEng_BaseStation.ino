#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include "soc/rtc_wdt.h"
//hspi
#define LORA_FREQUENCY 868.0  // LoRa frequency in Hz
#define LORA_CS 15            // 15//
#define LORA_RST 4            // LoRa reset pin//
#define LORA_GO_INT 27        // LoRa DIO0 interrupt pin//
#define LORA_SCK 14           // 14//
#define LORA_MISO 12          // 12//
#define LORA_MOSI 13          // 13//
//Mode indicators
#define BUTTON_PIN_1 32  // Button 1 pin
//Device IDs
byte drone_id = 0xFF;
byte base_id = 0x07;
// Network credentials
const char* ssid = "Emi's Laptop";
const char* password = "miniSCADA";
// Global variables
const int numNodes = 5;
int lastState, interval = 1;  // the previous state from the input pin
bool RequestDataMode, locationCollectionMode, dataCollectionMode, rssiCollectionMode, webserverMode, flag, flag1 = false;
int dataCount, rssiCount, currentState, x, Secs = 0;
String locas[numNodes][3], rssis[30][4], data[60][9], incoming, message, datad, rssid, locationString = "";
byte msgCount = 0;
unsigned long previousMillis, currentMillis = 0;
unsigned long int previoussecs, currentsecs = 0;
// Create an Event Source on /events
AsyncEventSource events("/events");
AsyncWebServer server(80);
SPIClass* hspi = NULL;

void setup() {
  Serial.begin(115200);

  hspi = new SPIClass(HSPI);

  pinMode(LORA_GO_INT, INPUT_PULLUP);
  pinMode(LORA_RST, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN_1, HIGH);

  // manual reset LoRa module
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);
  delay(10);

  hspi->begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPIFrequency(1E6);
  LoRa.setSPI(*hspi);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_GO_INT);
  if (!LoRa.begin(LORA_FREQUENCY)) {  // Set LoRa frequency to 868 MHz
    Serial.println("LoRa initialization failed. Check your connections!");
  }
  LoRa.setTxPower(20);
  Serial.println("LoRa and button initialized successfully!");
}

void loop() {
  currentState = digitalRead(BUTTON_PIN_1);
  handleButtons();  // Check button to switch modes

  currentMillis = millis();
  currentsecs = currentMillis / 1000;
  if ((unsigned long)(currentsecs - previoussecs) >= interval) {
    Secs = Secs + 1;
    if (Secs >= 5) {
      Secs = 0;
    }
    if ((Secs == 1) || (Secs == 4)) {
      if (RequestDataMode) {
        // Mode 1: Node location collection from drone
        if (locationCollectionMode) {
          Serial.println("Request Location Data");
          message = "1";
          sendMessage(message, drone_id, base_id);
        }
        // Mode 2: Data collection from drone
        if (dataCollectionMode) {
          Serial.println("Request Data");
          message = "2";
          sendMessage(message, drone_id, base_id);
        }
        // Mode 3: Data collection from slave nodes based on their respective locations
        if (rssiCollectionMode) {
          Serial.println("Request RSSI Data");
          message = "3";
          sendMessage(message, drone_id, base_id);
        }
      }
    }
    previoussecs = currentsecs;
  }

  if (RequestDataMode) {
    onReceive(LoRa.parsePacket());
  }

  if (webserverMode) {
    Serial.print("Post data");
    initWiFi();
    initSPIFFS();
    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send(SPIFFS, "/index.html", "text/html");
    });
    server.serveStatic("/", SPIFFS, "/");
    // Request for the latest sensor readings
    server.on("/readings", HTTP_GET, [](AsyncWebServerRequest* request) {
      //String json = getNodeData();
      String json = getNodeData();
      request->send(200, "application/json", json);
      json = String();
    });
    server.on("/rssi", HTTP_GET, [](AsyncWebServerRequest* request) {
      String json = getRSSIData();
      request->send(200, "application/json", json);
      json = String();
    });
    server.on("/node-locations", HTTP_GET, [](AsyncWebServerRequest* request) {
      String json = getLocData();
      //String json = locationString;
      request->send(200, "application/json", json);
      json = String();
    });
    events.onConnect([](AsyncEventSourceClient* client) {
      if (client->lastId()) {
        Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
      }
      // send event with message "hello!", id current millis
      // and set reconnect delay to 1 second
      client->send("hello!", NULL, millis(), 10000);
    });
    server.addHandler(&events);
    // Start server
    server.begin();

    events.send("ping", NULL, millis());
    events.send(getNodeData().c_str(), "new_readings", millis());


    webserverMode = false;
    Serial.println("WebServer Success");
  }
}

void onReceive(int packetSize) {
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  if (packetSize == 0) return;
  Serial.println("incoming");
  int Val1 = 0;
  int recipient = LoRa.read();        // recipient address
  byte sender = LoRa.read();          // sender address
  byte incomingMsgId = LoRa.read();   // incoming msg ID
  byte incomingLength = LoRa.read();  // incoming msg length
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (recipient != base_id) {
    Serial.println("This message is not for me.");
    LoRa.receive();
    incoming = "";
    return;  // skip rest of function
  }
  if (locationCollectionMode) {
    Serial.println(incoming);
    storeLocations(incoming);
    incoming = "";
    //locationString = incoming;
    LoRa.receive();
    locationCollectionMode = false;
    dataCollectionMode = true;
    rssiCollectionMode = false;
    flag = false;
  } else if (dataCollectionMode || flag) {
    Serial.println(incoming);
    Val1 = incoming.toInt();
    if (Val1 == 5) {
      storeData(datad);
      rssiCollectionMode = true;
      flag = false;
      datad = "";
    } else {
      datad += incoming;
      flag = true;
    }
    incoming = "";
    dataCollectionMode = false;
    LoRa.receive();
  } else if (rssiCollectionMode || flag1) {
    //Val1 = incoming.toInt();
    // if(Val1 == 5){
    //   storeRssi(rssid);
    //   flag1 = false;
    //   webserverMode = true;
    //   RequestDataMode = false;
    // } else{
    //   rssid += incoming;
    //   flag1 = true;
    // }
    storeRssi(incoming);      //no
    webserverMode = true;     //no
    RequestDataMode = false;  //no
    rssiCollectionMode = false;
    Serial.println(incoming);
    incoming = "";
    LoRa.receive();
  }
}

void storeLocations(String messages) {
  int nodeId;
  char longitudeStr[20], latitudeStr[20];  // Adjust buffer size as necessary
  int i = 0;
  int startIndex = 0;
  while (startIndex < messages.length() && sscanf(messages.c_str() + startIndex, "{Node%d;%[^;];%[^}]}", &nodeId, longitudeStr, latitudeStr) == 3) {
    locas[i][0] = "Node" + String(nodeId);
    locas[i][1] = String(longitudeStr);
    locas[i][2] = String(latitudeStr);
    Serial.println(locas[i][0] + " " + locas[i][1] + " " + locas[i][2]);
    i++;

    // Move to the next node in the string
    startIndex = messages.indexOf('}', startIndex) + 1;

    // Skip any leading whitespace or newlines
    while (startIndex < messages.length() && isspace(messages[startIndex])) {
      startIndex++;
    }
  }
}

void storeData(String messages) {
  Serial.println(messages);
  dataCount = 0;
  int nId, rID, SM, YY, MM, DD, HH, M, SS;
  float Temp, Pres, Hum, Lux;
  int startIndex = 0;

  while (startIndex < messages.length() && sscanf(messages.c_str() + startIndex, "{Node%d;%d;%d-%d-%d;%d:%d:%d;%f;%f;%f;%f;%d}", &nId, &rID, &YY, &MM, &DD, &HH, &M, &SS, &Temp, &Pres, &Hum, &Lux, &SM) == 13) {
    // Store the values in the array
    data[dataCount][0] = "Node" + String(nId);
    data[dataCount][1] = String(rID);
    data[dataCount][2] = String(YY) + "-" + String(MM) + "-" + String(DD);
    data[dataCount][3] = String(HH) + ":" + String(M) + ":" + String(SS) + ".000";
    data[dataCount][4] = String(Temp);
    data[dataCount][5] = String(Pres);
    data[dataCount][6] = String(Hum);
    data[dataCount][7] = String(Lux);
    data[dataCount][8] = String(SM);
    dataCount++;

    // Move to the next entry in the string
    startIndex = messages.indexOf('}', startIndex) + 1;

    // Skip any leading whitespace or newlines
    while (startIndex < messages.length() && isspace(messages[startIndex])) {
      startIndex++;
    }
  }
}

void storeRssi(String messages) {
  rssiCount = 0;
  int nodeId, Yy, Mm, Dd, Hh, Ms, Ss, rssi;
  int startIndex = 0;

  while (startIndex < messages.length() && sscanf(messages.c_str() + startIndex, "{Node%d;%d-%d-%d;%d:%d:%d;%d}", &nodeId, &Yy, &Mm, &Dd, &Hh, &Ms, &Ss, &rssi) == 8) {
    // Store the values in the array
    rssis[rssiCount][0] = "Node" + String(nodeId);
    rssis[rssiCount][1] = String(Yy) + "-" + String(Mm) + "-" + String(Dd);
    rssis[rssiCount][2] = String(Hh) + ":" + String(Ms) + ":" + String(Ss) + ".000";
    rssis[rssiCount][3] = String(rssi);
    rssiCount++;

    // Move to the next entry in the string
    startIndex = messages.indexOf('}', startIndex) + 1;

    // Skip any leading whitespace or newlines
    while (startIndex < messages.length() && isspace(messages[startIndex])) {
      startIndex++;
    }
  }
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  Serial.println("sending request to Drone, with message: " + outgoing);
  LoRa.beginPacket();             // start packet
  LoRa.write(otherNode);          // add destination address
  LoRa.write(MasterNode);         // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);
  LoRa.endPacket();  // finish packet and send it
  LoRa.receive();
  msgCount++;  // increment message ID
}

String getNodeData() {
  // request node data and sort

  DynamicJsonDocument doc(4096);  // Adjust the size according to your data

  // Iterate through each node
  for (int b = 0; b < dataCount; b++) {
    // Get the node ID
    const char* nodeId = data[b][0].c_str();

    // Check if the node ID already exists in the document
    JsonArray nodeData;
    if (!doc.containsKey(nodeId)) {
      // If not, create a new array for the node
      nodeData = doc.createNestedArray(nodeId);
    } else {
      // If it exists, get the existing array
      nodeData = doc[nodeId];
    }

    // Create a new object for the reading
    JsonObject reading = nodeData.createNestedObject();

    // Add the reading data
    reading["date"] = data[b][2];
    reading["time"] = data[b][3];
    reading["temperature"] = data[b][4];
    reading["pressure"] = data[b][5];
    reading["humidity"] = data[b][6];
    reading["lux"] = data[b][7];
    reading["soilMoisture"] = data[b][8];
  }

  // Serialize the JSON document to a string
  String jsonString;
  serializeJson(doc, jsonString);

  // Print the JSON string for debugging
  Serial.println(jsonString);

  // Return the JSON string
  return jsonString;
}

String getRSSIData() {
  //Request rssi data from drone first

  DynamicJsonDocument doc(4096);  // Adjust the size according to your data

  // Iterate through each node
  for (int b = 0; b < rssiCount; b++) {
    // Get the node ID
    const char* nodeId = rssis[b][0].c_str();

    // Check if the node ID already exists in the document
    JsonArray nodeData;
    if (!doc.containsKey(nodeId)) {
      // If not, create a new array for the node
      nodeData = doc.createNestedArray(nodeId);
    } else {
      // If it exists, get the existing array
      nodeData = doc[nodeId];
    }

    // Create a new object for the reading
    JsonObject rssi = nodeData.createNestedObject();

    // Add the reading data
    rssi["date"] = rssis[b][1];
    rssi["time"] = rssis[b][2];
    rssi["rssi"] = rssis[b][3];
  }

  String jsonVal;
  serializeJson(doc, jsonVal);

  // Print the JSON string for debugging
  Serial.println(jsonVal);

  // Return the JSON string
  return jsonVal;
}

String getLocData() {
  //LORA Request location data from drone first

  DynamicJsonDocument doc(4096);  // Adjust the size according to your data

  // Iterate through each node
  for (int b = 0; b < 5; b++) {

    String nodeid = "Node" + locas[b][0];

    // Get the node ID
    const char* nodeId = nodeid.c_str();

    // Check if the node ID already exists in the document
    JsonArray nodeData1;
    if (!doc.containsKey(nodeId)) {
      // If not, create a new array for the node
      nodeData1 = doc.createNestedArray(nodeId);
    } else {
      // If it exists, get the existing array
      nodeData1 = doc[nodeId];
    }

    // Create a new object for the reading
    JsonObject locs = nodeData1.createNestedObject();

    // Add the reading data
    locs["id"] = locas[b][0];
    locs["longitude"] = locas[b][1];
    locs["latitude"] = locas[b][2];
  }

  String jsonVal1;
  serializeJson(doc, jsonVal1);

  // Print the JSON string for debugging
  Serial.println(jsonVal1);

  // Return the JSON string
  return jsonVal1;
}

void handleButtons() {
  if (lastState == HIGH && currentState == LOW) {
    RequestDataMode = true;
    locationCollectionMode = true;
    Serial.println("Request Data mode initiated.");
    delay(1000);  // Debouncing delay
    lastState = currentState;
  }
  //digitalWrite(BUTTON_PIN_1, HIGH);
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }
}
// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  bool connected = WL_CONNECTED;
  int n = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
    n++;
    if (n == 30) {
      connected = true;
      n = 0;
    }
  }
  Serial.println(WiFi.localIP());
}
