#include <LoRa.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include "soc/rtc_wdt.h"
#include <TinyGPS++.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <iostream>
#include <ArduinoJson.h>  // Include ArduinoJson library
//hspi
#define LORA_FREQUENCY 868.0  // LoRa frequency in Hz
#define LORA_CS 15            // 15//
#define LORA_RST 4            // LoRa reset pin//
#define LORA_GO_INT 27        // LoRa DIO0 interrupt pin//
#define LORA_SCK 14           // 14//
#define LORA_MISO 12          // 12//
#define LORA_MOSI 13          // 13//
//vspi
#define SD_CS 5     //5
#define SD_CLK 18   //18
#define SD_MISO 19  //19
#define SD_MOSI 23  //23
#define FILE_NAME "/datadrone.txt"
#define FILE_NAME1 "/locations.txt"
#define FILE_NAME2 "/rssi.txt"
//Mode indicators
#define BUTTON_PIN_1 32         // Button 1 pin Location
#define LED_PIN_1 33            // Yellow Location
#define LED_PIN_2 25            // GREEN Data
#define LED_PIN_3 26            // RED Stop
#define EARTH_RADIUS 6371000.0  // Earth radius in meters
#define RXD2 16
#define TXD2 17
#define TX_POWER 20  // Transmission power in dBm
//Node IDs
byte master_id = 0xFF;
byte device_ids[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
// Network credentials
const char *ssid = "Emi's Laptop";
const char *password = "miniSCADA";
// Drone parameters
double droneStartLocation[2] = { -33.94735, 18.87066 };  // Define the drone starting location
double droneCurLocation[2] = {-33.95088, 18.86847};                   // Define the drone starting location
// Define Operation States
bool locationCollectionMode, dataCollectionMode, flightPathMode, webserverMode, BaseStation = false;
bool flag = true;
// Global variables
const int numNodes = 5;
byte nearestNodeI, msgCount = 0;
String incoming, message, SenderNode, locationsString, dataString, rssiString, dayStamp, timeStamp = "";
double Longitude, Latitude;
unsigned long previousMillis = 0;
unsigned long int previoussecs, currentsecs = 0;
int interval, lastState = 1;  // updated every 1 second
int Secs, r, x, dataCount, dataIndex, rssiCount, currentState, Val = 0;
double locations[numNodes][3];
String data[60][9], rssis[30][4], locas[numNodes][3];
int orderedDeviceIds[numNodes + 1];
// Create an Event Source on /events
AsyncWebServer server(80);
AsyncEventSource events("/events");
SPIClass *hspi = NULL;
SPIClass *vspi = NULL;
SPIClass SPI2(HSPI);
TinyGPSPlus gps;
HardwareSerial neogps(1);

void setup() {
  Serial.begin(115200);

  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(LORA_GO_INT, INPUT_PULLUP);
  pinMode(LORA_RST, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);

  vspi->begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  delay(4000);
  if (!SD.begin(SD_CS, *vspi, 4000000)) {
    Serial.println("SD card initialization failed. Check your wiring.");
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
  }
  File file = SD.open("/datadrone.txt");
  if (!file) {
    Serial.println("File doens't exist, Creating file...");
    writeFile(SD, "/datadrone.txt", "");
  } else {
    Serial.println("File already exists");
  }
  file.close();
  File file1 = SD.open("/locations.txt");
  if (!file1) {
    Serial.println("File doens't exist, Creating file...");
    writeFile(SD, "/locations.txt", "");
  } else {
    Serial.println("File already exists");
  }
  file.close();
  File file2 = SD.open("/rssi.txt");
  if (!file2) {
    Serial.println("File doens't exist, Creating file...");
    writeFile(SD, "/rssi.txt", "");
  } else {
    Serial.println("File already exists");
  }
  file.close();

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

  digitalWrite(LED_PIN_3, HIGH);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);

  Serial.println("Free Heap: " + String(ESP.getFreeHeap()));

  LoRa.setTxPower(TX_POWER);
  Secs = 0;

  Serial.println("LoRa, SD card, and buttons initialized successfully!");
  LoRa.receive();
}

void loop() {
  currentState = digitalRead(BUTTON_PIN_1);
  handleButtons();  // Check button to switch modes
  if (webserverMode) {
    readNodeData();
    Serial.print("Post read data");
    initWiFi();
    initSPIFFS();
    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html");
    });
    server.serveStatic("/", SPIFFS, "/");
    // Request for the latest sensor readings
    server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request) {
      String json = getNodeData();
      request->send(200, "application/json", json);
      json = String();
    });
    server.on("/rssi", HTTP_GET, [](AsyncWebServerRequest *request) {
      String json = getRSSIData();
      request->send(200, "application/json", json);
      json = String();
    });
    server.on("/node-locations", HTTP_GET, [](AsyncWebServerRequest *request) {
      String json = getLocData();
      request->send(200, "application/json", json);
      json = String();
    });
    events.onConnect([](AsyncEventSourceClient *client) {
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
    delay(2000);

    events.send("ping", NULL, millis());
    events.send(getNodeData().c_str(), "new_readings", millis());

    webserverMode = false;
    Serial.println("WebServer Success");
  }
  if (flightPathMode) {
    r = 0;
    readNodeLocations();
    getLocation();
    droneCurLocation[0] = Longitude;
    droneCurLocation[1] = Latitude;
    Serial.println("Drone Current Location:" + String(droneCurLocation[0],6) + "," + String(droneCurLocation[1],6));
    //Serial.print("Node ");
    for (int i = 0; i < numNodes; i++) {
      Serial.print("Node ");
      Serial.print(locations[i][0]);
      Serial.print(" - Longitude: ");
      Serial.print(locations[i][1], 6);
      Serial.print(", Latitude: ");
      Serial.println(locations[i][2], 6);
    }
    findShortestPath(droneStartLocation, locations, numNodes, orderedDeviceIds);
    nearestNodeI = findNearestNode(droneCurLocation[0], droneCurLocation[1], locations, numNodes);
    Serial.print("Nearest Node: ");
    Serial.println(device_ids[nearestNodeI]);
    // Print the ordered device IDs for verification
    Serial.print("Location-based Polling Sequence Ordered Device IDs: ");
    for (int i = 0; i < numNodes; ++i) {
      orderedDeviceIds[i] = orderedDeviceIds[i] + 1;
      Serial.print(orderedDeviceIds[i], HEX);
      Serial.print(",");
    }
    Serial.println();
    dataCollectionMode = true;
    flightPathMode = false;
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    Secs = Secs + 1;
    //Serial.println(Secs);
    if (Secs > 16) {
      Secs = 0;
      r = r + 1;
      if (r == numNodes) {
        r = 0;
        flag = true;
      }
    }
    // Priority poll nearest node
    if ((Secs == 15) || (Secs == 16)) {
      
      // Mode 1: Individual slave Node GPS location collection
      if (locationCollectionMode) {
        message = "1";
        sendMessage(message, master_id, device_ids[nearestNodeI]);
      }
      // Mode 2: Data collection from slave nodes based on their respective locations
      if (dataCollectionMode) {
        if (flag) {
          getLocation();
          nearestNodeI = findNearestNode(droneCurLocation[0], droneCurLocation[1], locations, numNodes);
          Serial.print("Rechecking Current Nearest Node ID: ");
          Serial.println(device_ids[nearestNodeI]);
          message = "2";
          sendMessage(message, master_id, device_ids[nearestNodeI]);
        }
      }
    }
    // Sequential query
    if ((Secs == 7) || (Secs == 8)) {
      // Mode 1: Individual slave Node GPS location collection
      if (locationCollectionMode) {
        message = "1";
        sendMessage(message, master_id, device_ids[r]);
      }
      // Mode 2: Data collection from slave nodes based on their respective locations
      if (dataCollectionMode) {
        if (flag) {
          message = "2";
          sendMessage(message, master_id, orderedDeviceIds[r]);
        }
      }
    }
  }
  if (BaseStation) {
    if (Val == 1) {  //send node locations
      readNodeLocations();
      sendMessage(locationsString, master_id, 0x07);
      locationsString = "";
    }
    if (Val == 2) {  //send node data
      readNodeData();
      sendMessage(dataString, master_id, 0x07);
      dataString = "";
    }
    if (Val == 3) {  //send rssi data
      readRssiData();
      sendMessage(rssiString, master_id, 0x07);
      rssiString = "";
    }
    BaseStation = false;
  }
  onReceive(LoRa.parsePacket());
}

void getLocation() {
  for (int k = 0; k < 20; k++) {
    gps.encode(neogps.read());
    //if(gps.location.isUpdated()){
    //Serial.print("Latitude= ");
    Latitude = gps.location.lat();
    //Serial.print(Latitude, 6);
    //Serial.print(" Longitude= ");
    Longitude = gps.location.lng();
    //Serial.println(gps.location.lng(), 6);
    //}
    dayStamp = gps.date.value();
    //Serial.print("Raw date DDMMYY = ");
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();
    int hour = gps.time.hour() + 2;  //adjust UTC
    int minute = gps.time.minute();
    int second = gps.time.second();
    if (hour > 23) {
      hour = hour - 24;
      day = day + 1;
      if (day > 30) {
        if (month == 4 || month == 6 || month == 9 || month == 11) {
          day = 1;
          month = month + 1;
        }
      }
      if (day > 31) {
        if (month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10) {
          day = 1;
          month = month + 1;
        }
      }
    }
    dayStamp = String(year) + "-" + String(month) + "-" + String(day);
    //Serial.println(dayStamp);
    //Serial.print("Raw time in HHMMSS = ");
    //Serial.println(dayStamp);
    //Serial.print("Raw time in HHMMSS = ");
    timeStamp = String(hour) + ":" + minute + ":" + second;
    //Serial.println(timeStamp);
    //Serial.println();
  }
}

void readNodeLocations() {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  // Open the data file
  File dataFile = SD.open(FILE_NAME1);
  // Check if the file is opened successfully
  if (dataFile) {
    int nodeCount = 0;
    // Read data from the file until there's nothing left or until we've read locations for all nodes
    while (dataFile.available() && nodeCount < numNodes) {
      // Read a line from the file
      String line = dataFile.readStringUntil('\n');
      locationsString += line;
      // Parse the line into node id, longitude, and latitude
      int nodeId;
      char longitudeStr[20], latitudeStr[20];  // Adjust buffer size as necessary
      sscanf(line.c_str(), "{Node%d;%[^;];%[^}]}", &nodeId, &longitudeStr, &latitudeStr);
      //Serial.println(longitudeStr);
      locas[nodeCount][0] = String(nodeId);
      locas[nodeCount][1] = longitudeStr;
      locas[nodeCount][2] = latitudeStr;
      double longitude = atof(longitudeStr);
      double latitude = atof(latitudeStr);
      // Store the values in the array
      locations[nodeCount][0] = nodeId;
      locations[nodeCount][1] = longitude;
      locations[nodeCount][2] = latitude;

      nodeCount++;
    }
    // Close the file
    dataFile.close();
  } else {
    // If the file didn't open, print an error message
    Serial.println("Error opening locations.txt");
  }
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
  Serial.print("Polling Node ");
  Serial.println(otherNode);

  int packetSize = 240;  // Maximum payload size per packet

  // Calculate the number of packets needed
  int numPackets = (outgoing.length() + packetSize - 1) / packetSize;

  //Serial.println(numPackets);

  // Loop through each packet
  for (int i = 0; i < numPackets; i++) {
    LoRa.beginPacket();                                                    // start packet
    LoRa.write(otherNode);                                                 // add destination address
    LoRa.write(MasterNode);                                                // add sender address
    LoRa.write(msgCount);                                                  // add message ID
    int len = min(packetSize, (int)outgoing.length() - i * packetSize);    // Calculate length for this packet
    LoRa.write(len);                                                       // add payload length
    LoRa.print(outgoing.substring(i * packetSize, i * packetSize + len));  // Add payload chunk
    LoRa.endPacket();                                                      // finish packet and send it
    LoRa.receive();
    //Serial.println(outgoing.substring(i * packetSize, i * packetSize + len));
    delay(900);
  }

  if (numPackets > 1) {
    int ack = 5;
    LoRa.beginPacket();                // start packet
    LoRa.write(otherNode);             // add destination address
    LoRa.write(MasterNode);            // add sender address
    LoRa.write(msgCount);              // add message ID
    LoRa.write(String(ack).length());  // add payload length
    LoRa.print(String(ack));           // Add payload chunk
    LoRa.endPacket();                  // finish packet and send it
    LoRa.receive();
  }

  numPackets = 0;

  msgCount++;  // increment message ID
}

void onReceive(int packetSize) {
  rtc_wdt_protect_off();
  rtc_wdt_disable();
  if (packetSize == 0) return;
  Serial.println("incoming");
  int recipient = LoRa.read();  // recipient address
  byte sender = LoRa.read();    // sender address
  if (sender == 0X07)
    BaseStation = true;
  if (sender == 0X01)
    SenderNode = "Node1";
  if (sender == 0X02)
    SenderNode = "Node2";
  if (sender == 0X03)
    SenderNode = "Node3";
  if (sender == 0X04)
    SenderNode = "Node4";
  if (sender == 0X05)
    SenderNode = "Node5";
  byte incomingMsgId = LoRa.read();   // incoming msg ID
  byte incomingLength = LoRa.read();  // incoming msg length
  incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (BaseStation) {
    Serial.print("Received LoRa message from Base Station: ");
    Serial.println(incoming);
    Val = incoming.toInt();
  } else if (locationCollectionMode) {
    // print RSSI of packet
    String rssi1 = "{" + String(SenderNode) + ";" + String(dayStamp) + ";" + String(timeStamp) + ";" + String(LoRa.packetRssi()) + "}\n";
    Serial.print("Storing RSSi data from ");
    Serial.print(String(SenderNode));
    Serial.print(": ");
    Serial.println(rssi1);
    storeRssiOnSDCard(rssi1);
    incoming = "{" + String(SenderNode) + ";" + String(incoming) + "}";
    Serial.print("Storing location data from ");
    Serial.print(String(SenderNode));
    Serial.print(": ");
    Serial.println(incoming);
    storeLocationOnSDCard(incoming);
    LoRa.receive();
  } else if (dataCollectionMode) {
    flag = false;
    // print RSSI of packet
    String rssi = "{" + String(SenderNode) + ";" + String(dayStamp) + ";" + String(timeStamp) + ";" + String(LoRa.packetRssi()) + "}\n";
    
    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Message: " + incoming);
    flag = false;

    int Ack = incoming.toInt();

    if (Ack == 5) {
      message = "3";
      sendMessage(message, master_id, sender);
      delay(1000);
      sendMessage(message, master_id, sender);
      Serial.println("Successful commss");
      LoRa.receive();
      flag = true;
    } else {
      Serial.print("Storing RSSi data from ");
      Serial.print(String(SenderNode));
      Serial.print(": ");
      Serial.println(rssi);
      storeRssiOnSDCard(rssi);
      Serial.print("Storing data from ");
      Serial.print(String(SenderNode));
      Serial.print(": ");
      Serial.println(incoming);
      storeMessageOnSDCard(incoming);
    }
  }
}

void storeMessageOnSDCard(String message) {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  // Open the file in write mode
  appendFile(SD, FILE_NAME, message.c_str());
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}

void storeRssiOnSDCard(String message) {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  // Open the file in write mode
  appendFile(SD, FILE_NAME2, message.c_str());
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}

void storeLocationOnSDCard(String message) {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  // Open the file in read and write mode
  File dataFile = SD.open(FILE_NAME1, FILE_READ);
  String newData = "";
  bool nodeFound = false;
  // Check if the file is opened successfully
  if (dataFile) {
    // Read data from the file and search for the node ID
    while (dataFile.available()) {
      String line = dataFile.readStringUntil('\n');
      int nodeId;
      sscanf(line.c_str(), "{Node%d", &nodeId);
      // If the node ID matches, overwrite the data
      if (nodeId == extractNodeIdFromMessage(message)) {
        newData += message + "\n";
        nodeFound = true;
      } else {
        newData += line + "\n";
      }
    }
    dataFile.close();
  } else {
    // If the file didn't open, print an error message
    Serial.println("Error opening locations.txt");
  }
  // If the node ID was not found, append the new data
  if (!nodeFound) {
    newData += message + "\n";
  }
  // Open the file in write mode to save the updated data
  File dataFileWrite = SD.open(FILE_NAME1, FILE_WRITE);
  if (dataFileWrite) {
    dataFileWrite.print(newData);
    dataFileWrite.close();
  } else {
    Serial.println("Error writing to locations.txt");
  }
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}
// Helper function to extract the node ID from the message
int extractNodeIdFromMessage(String message) {
  int nodeId;
  sscanf(message.c_str(), "{Node%d;", &nodeId);
  return nodeId;
}

void handleButtons() {
  if (lastState == HIGH && currentState == LOW) {
    x = x + 1;
    // Button 1: Initiate location collection mode
    if (x == 1) {
      locationCollectionMode = true;
      dataCollectionMode = false;
      flightPathMode = false;
      webserverMode = false;
      Serial.println();
      Serial.println("Location collection mode initiated.");
      delay(1000);  // Debouncing delay
      digitalWrite(LED_PIN_3, LOW);
      digitalWrite(LED_PIN_2, LOW);
      digitalWrite(LED_PIN_1, HIGH);
    }
    // Button 2: Initiate data collection based on slave node locations
    if (x == 2) {
      dataCollectionMode = false;
      locationCollectionMode = false;
      flightPathMode = true;
      webserverMode = false;
      Serial.println();
      Serial.println("Data collection based on slave node locations initiated.");
      delay(1000);  // Debouncing delay
      digitalWrite(LED_PIN_3, LOW);
      digitalWrite(LED_PIN_2, HIGH);
      digitalWrite(LED_PIN_1, LOW);
    }
    // Button 3: Stop all data collection
    if (x == 3) {
      flightPathMode = false;
      dataCollectionMode = false;
      locationCollectionMode = false;
      webserverMode = false;
      Serial.println();
      Serial.println("Stop All Data Collection");
      delay(1000);  // Debouncing delay
      digitalWrite(LED_PIN_3, HIGH);
      digitalWrite(LED_PIN_2, LOW);
      digitalWrite(LED_PIN_1, LOW);
    }
    // Button 4: Data to Webserver
    if (x == 4) {
      flightPathMode = false;
      dataCollectionMode = false;
      locationCollectionMode = false;
      webserverMode = true;
      Serial.println();
      Serial.println("Data to Webserver");
      delay(1000);  // Debouncing delay
      digitalWrite(LED_PIN_3, LOW);
      digitalWrite(LED_PIN_2, LOW);
      digitalWrite(LED_PIN_1, LOW);
      x = 0;
    }
  }
}
// Function to calculate distance between two points using Haversine formula
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float dlat = radians(lat2 - lat1);
  float dlon = radians(lon2 - lon1);
  float a = sin(dlat / 2.0) * sin(dlat / 2.0) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2.0) * sin(dlon / 2.0);
  float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  float distance = 6371000.0 * c;  // Radius of Earth in meters
  return distance;
}
// Function to find the index of the nearest node to the drone's current location
byte findNearestNode(float droneLatitude, float droneLongitude, double locations[][3], int numNodes) {
  float minDistance = 999999.0;  // Initialize with a large value
  byte nearestNodeIndex = 0;
  for (byte i = 0; i < numNodes; i++) {
    float distance = calculateDistance(droneLatitude, droneLongitude, locations[i][1], locations[i][2]);
    if (distance < minDistance) {
      minDistance = distance;
      nearestNodeIndex = i;
    }
  }
  return nearestNodeIndex;
}
// Function to calculate Haversine distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dlat = radians(lat2 - lat1);
  double dlon = radians(lon2 - lon1);
  double a = sq(sin(dlat / 2.0)) + cos(radians(lat1)) * cos(radians(lat2)) * sq(sin(dlon / 2.0));
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS * c;
}
// Function to find the approximate shortest path using nearest neighbor
void findShortestPath(double startLocation[2], double nodes[][3], int nodeCount, int path[]) {
  // Initialize the visited array
  bool visited[nodeCount];
  for (int i = 0; i < nodeCount; ++i) {
    visited[i] = false;
  }
  // Initialize the path with the starting node
  path[0] = 0;
  visited[0] = true;
  // Find the nearest neighbor for each step
  for (int i = 1; i < nodeCount; ++i) {
    int current = path[i - 1];
    double minDistance = INFINITY;
    int nearestNeighbor = -1;
    for (int j = 0; j < nodeCount; ++j) {
      if (!visited[j]) {
        double distance = haversine(nodes[current][1], nodes[current][2], nodes[j][1], nodes[j][2]);
        if (distance < minDistance) {
          minDistance = distance;
          nearestNeighbor = j;
        }
      }
    }
    // Move to the nearest neighbor
    path[i] = nearestNeighbor;
    visited[nearestNeighbor] = true;
  }
  // Add the starting location to the end of the path
  path[nodeCount] = 0;
}

String getNodeData() {
  DynamicJsonDocument doc(4096);  // Adjust the size according to your data

  // Iterate through each node
  for (int b = 0; b < dataCount; b++) {
    // Get the node ID
    const char *nodeId = data[b][0].c_str();

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
  readRssiData();
  DynamicJsonDocument doc(4096);  // Adjust the size according to your data

  // Iterate through each node
  for (int b = 0; b < rssiCount; b++) {
    // Get the node ID
    const char *nodeId = rssis[b][0].c_str();

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
  readNodeLocations();
  DynamicJsonDocument doc(4096);  // Adjust the size according to your data

  // Iterate through each node
  for (int b = 0; b < 5; b++) {

    String nodeid = "Node" + locas[b][0];

    // Get the node ID
    const char *nodeId = nodeid.c_str();

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

void readRssiData() {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  // Open the data file
  File dataFile = SD.open(FILE_NAME2);
  // Check if the file is opened successfully
  if (dataFile) {
    rssiCount = 0;
    // Read data from the file until there's nothing left or until we've read locations for all nodes
    while (dataFile.available()) {
      // Read a line from the file
      String line = dataFile.readStringUntil('\n');
      rssiString += line;
      // Parse the line into node id, longitude, and latitude
      int nodeId, Yy, Mm, Dd, Hh, Ms, Ss, rssi;
      sscanf(line.c_str(), "{Node%d;%d-%d-%d;%d:%d:%d;%d}", &nodeId, &Yy, &Mm, &Dd, &Hh, &Ms, &Ss, &rssi);
      // Store the values in the array
      rssis[rssiCount][0] = "Node" + String(nodeId);
      rssis[rssiCount][1] = String(Yy) + "-" + String(Mm) + "-" + String(Dd);
      rssis[rssiCount][2] = String(Hh) + ":" + String(Ms) + ":" + String(Ss) + ".000";
      rssis[rssiCount][3] = String(rssi);
      rssiCount++;
    }
    // Close the file
    dataFile.close();
  } else {
    // If the file didn't open, print an error message
    Serial.println("Error opening rssi.txt");
  }
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}

void readNodeData() {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);

  // Open the data file
  File dataFile = SD.open(FILE_NAME);
  dataCount = 0;

  int nId, rID, SM, YY, MM, DD, HH, M, SS;
  float Temp, Pres, Hum, Lux;
  String date, time;

  // Check if the file is opened successfully
  if (dataFile) {
    // Read data from the file until there's nothing left or until we've read locations for all nodes
    String line;
    while ((line = dataFile.readStringUntil('\n')) != "") {
      dataString += line;

      // Parse the line into data components
      sscanf(line.c_str(), "{Node%d;%d;%d-%d-%d;%d:%d:%d;%f;%f;%f;%f;%d}\n",
             &nId, &rID, &YY, &MM, &DD, &HH, &M, &SS, &Temp, &Pres, &Hum, &Lux, &SM);

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

      //dataString += String(data[dataCount][0]) +","+String(data[dataCount][1]) +","+String(data[dataCount][2]) +","+data[dataCount][3] +","+data[dataCount][4] +","+data[dataCount][5] +","+data[dataCount][6] +","+data[dataCount][7] +","+data[dataCount][8]);
      dataCount++;
    }
    Serial.println(dataString);

    // Close the file
    dataFile.close();
  } else {
    // If the file didn't open, print an error message
    Serial.println("Error opening datadrone.txt");
  }

  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}
// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char *path, const char *message) {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
}
// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
  digitalWrite(LORA_CS, LOW);
  digitalWrite(SD_CS, HIGH);
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
