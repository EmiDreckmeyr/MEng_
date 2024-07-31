#include <SPI.h>
#include <LoRa.h>
#include <esp_sleep.h>
#include <SD.h>
#include "FS.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <TinyGPS++.h>
#include "soc/rtc_wdt.h"
#include <esp_task_wdt.h>

#define LORA_FREQUENCY 868.0  // LoRa frequency in Hz
#define LORA_CS 13            // LoRa chip select pin
#define LORA_RST 4            // LoRa reset pin
#define LORA_GO_INT 27        // LoRa DIO0 interrupt pin
#define LORA_SCK 14           // LoRa chip select pin
#define LORA_MISO 2           // LoRa reset pin
#define LORA_MOSI 15          // LoRa DIO0 interrupt pin
#define SD_CS 5
#define SD_CLK 18
#define SD_MISO 19
#define SD_MOSI 23
#define switchpower 12
#define rain_sensor 35
#define soilmoisturePin 36
#define RXD2 17
#define TXD2 16
#define SEALEVELPRESSURE_HPA (1013.25)
#define AirValue (3670)
#define WaterValue (803)
#define FILE_NAME "/data.txt"

Adafruit_BME280 bme;
BH1750 lightMeter;
TinyGPSPlus gps;
HardwareSerial neogps(1);

byte device_id = 0x05;
byte master_id = 0xFF;
String dataMessage = "hi";
int indexing = 0;
float temperature, pressure, altitude, humidity, lux;
int rainfall, soilmoisturepercent, soilMoistureValue;
double dewPoint;
String dayStamp, timeStamp;
byte val1, val2, val3;
long int t1, t2;
int day, month, year;
int Val = 0;
int hour = 19;
RTC_DATA_ATTR int minute = 22;
RTC_DATA_ATTR int second = 27;
// Define deep sleep options
uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
// Sleep for 10 minutes = 600 seconds
uint64_t TIME_TO_SLEEP_1 = 10;
uint64_t TIME_TO_SLEEP_2 = 300;
String sensorDataBuff = "";
// Save reading number on RTC memory
RTC_DATA_ATTR int msgCount = 0;
RTC_DATA_ATTR int bootCount = 0;  // Store boot count in RTC memory
RTC_DATA_ATTR int dataCount = 0;  // Store boot count in RTC memory
RTC_DATA_ATTR char sensorDataBuffer[3000];
RTC_DATA_ATTR int readingID = 1;
RTC_DATA_ATTR volatile bool LORA = false;
RTC_DATA_ATTR volatile bool SDCARD = true;
RTC_DATA_ATTR volatile bool LORA_RESTART = false;
RTC_DATA_ATTR volatile bool READRESP = false;
RTC_DATA_ATTR double Longitude, Latitude;
SPIClass *vspi = NULL;  // VSPI sd

void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(10);
  //delay(800);

  print_wakeup_reason();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  rtc_wdt_protect_off();
  rtc_wdt_disable();
  WRITE_PERI_REG(RTC_CNTL_WDTWPROTECT_REG, 0x68);    // Disable write protection of RTC WDT registers
  WRITE_PERI_REG(RTC_CNTL_WDTFEED_REG, 0x50D83AA1);  // Feed dog
  WRITE_PERI_REG(RTC_CNTL_WDTCONFIG0_REG, 0);        // Disable WDT
  //Serial.println("Watchdog timer disabled.");

  pinMode(switchpower, OUTPUT);
  digitalWrite(switchpower, LOW);
  pinMode(LORA_GO_INT, INPUT_PULLUP);
  pinMode(LORA_RST, OUTPUT);
  pinMode(rain_sensor, INPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);

  if (LORA_RESTART) {
    SDCARD = false;
  }

  digitalWrite(LORA_RST, HIGH);
  delay(10);
  digitalWrite(LORA_RST, LOW);
  delay(10);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_GO_INT);
  if (!LoRa.begin(LORA_FREQUENCY)) {  // Set LoRa frequency to 868 MHz
    Serial.println("LoRa initialization failed. Check your connections!");
  }

  LoRa.setTxPower(20);

  Serial.println("LoRa initialization gooood!");

  if (SDCARD) {
    Wire.begin();
    lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
    bme.begin();
    neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
    vspi = new SPIClass(VSPI);
    vspi->begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
      Serial.println("Card Mount Failed");
    }
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
    }
    Serial.println("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
      Serial.println("ERROR - SD card initialization failed!");
    }
    File file = SD.open("/data.txt");
    if (!file) {
      Serial.println("File doens't exist, Creating file...");
      writeFile(SD, "/data.txt", "");
    } else {
      Serial.println("File already exists");
    }
    file.close();
  }

  bootCount++;
  dataCount++;
  Serial.println("Boot count: " + String(bootCount));
  //Serial.println("Data: ");
  //Serial.println(String(sensorDataBuffer));
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  //esp_sleep_disable_wakeup_source(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);

  //LoRa.onReceive(onLoRaMessage);
  LoRa.receive();
}

void loop() {
  if (LORA_RESTART) {
    LORA_RESTART = false;
    SDCARD = false;
    LORA = false;
    //dataMessage = String(Longitude, 6) +";"+ String(Latitude, 6);
    //Serial.println("Sent: " + dataMessage);
    LoRa.receive();
    Serial.println("LORA restart. Going back to sleep");
    esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_2 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
  if (LORA) {
    LoRa.onReceive(onLoRaMessage);
    //onLoRaMessage(LoRa.parsePacket());
    indexing++;
    if (indexing == 90000) {
      indexing = 0;
      Serial.println("LORA time out, go to sleep.");
      LORA = false;
      esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_2 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
  }
  if (SDCARD) {
    digitalWrite(switchpower, HIGH);
    getReadings();
    //getTimeStamp();
    dayStamp = String(2024) + "-" + String(07) + "-" + String(23);
    timeStamp = String(hour) + ":" + String(minute) + ":" + String(second);
    logSDCard();
    readingID++;
    SD.end();
    SDCARD = false;
    digitalWrite(switchpower, LOW);
    LoRa.receive();
    LORA_RESTART = true;
    Serial.println("DONE LOGGING! Going to sleep...");
    esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_1 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
  if (READRESP) {
    esp_task_wdt_reset();
    if (Val == 1) {
      //t1 = millis();
      // Transmit sensor node location as LoRa message
      dataMessage = String(Longitude, 6) + ";" + String(Latitude, 6);
      //sendMessage(dataMessage, device_id, master_id);
      LoRa.beginPacket();
      LoRa.write(master_id);             // add destination address
      LoRa.write(device_id);             // add sender address
      LoRa.write(msgCount);              // add message ID
      LoRa.write(dataMessage.length());  // add payload length
      LoRa.print(String(dataMessage));
      LoRa.endPacket();
      Serial.println("Sent: " + dataMessage);
      LoRa.receive();
      msgCount++;
      Val = 0;
      //long int t2 = millis();
      //Serial.print("Time taken by the task: ");
      //Serial.print(t2 - t1);
      //Serial.println(" milliseconds");
      READRESP = false;
      // Put the ESP32 back to sleep if needed
      LORA = false;
      Serial.println("Going back to sleep");
      Serial.println();
      esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_2 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    if (Val == 2) {
      sensorDataBuff = String(sensorDataBuffer);
      sendMessage(sensorDataBuff, device_id, master_id);
      // LoRa.beginPacket();
      // LoRa.write(master_id);              // add destination address
      // LoRa.write(device_id);             // add sender address
      // LoRa.write(msgCount);                 // add message ID
      // LoRa.write(sensorDataBuff.length());        // add payload length
      // LoRa.print(sensorDataBuff);             // add payload
      // LoRa.endPacket();
      Serial.println("Sent: " + String(sensorDataBuffer));
      //Serial.println(msgCount);
      //LoRa.receive();
      //msgCount++;
      Val = 0;
      LORA = false;
      sensorDataBuff = "";
      READRESP = false;
      Serial.println("Going back to sleep");
      Serial.println();
      esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_2 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    if (Val == 3) {
      sensorDataBuff = "";
      memset(sensorDataBuffer, '\0', sizeof(sensorDataBuffer));
      dataCount = 0;
      // Put the ESP32 back to sleep if needed
      Serial.println("Success comms, going to sleep: ");
      Serial.println();
      LORA = false;
      Val = 0;
      READRESP = false;
      esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_2 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    READRESP = false;
  }
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      LORA = true;
      SDCARD = false;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      SDCARD = true;
      LORA = false;
      break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void onLoRaMessage(int packetSize) {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  if (packetSize == 0) return;
  indexing = 0;
  Serial.println("Incoming Request From Drone ");
  // This function will be called when a LoRa message is received

  // Read the LoRa message
  String receivedData = "";
  int recipient = LoRa.read();        // recipient address
  byte sender = LoRa.read();          // sender address
  byte incomingMsgId = LoRa.read();   // incoming msg ID
  byte incomingLength = LoRa.read();  // incoming msg length
  while (LoRa.available()) {
    receivedData += (char)LoRa.read();
  }
  if (incomingLength != receivedData.length()) {  // check length for error
    Serial.println("error message length.");
  }
  //Serial.println(String(sender));
  if (recipient != device_id) {
    Serial.println("This message is not for me.");
    LORA = false;
    LoRa.receive();
    Serial.println("Going back to sleep");
    Serial.println();
    esp_sleep_enable_ext1_wakeup(BIT(LORA_GO_INT), ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP_2 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
    return;  // skip rest of function
  } else {
    // Process the received LoRa message
    Serial.print("Received LoRa message: ");
    Serial.println(receivedData);
    Val = receivedData.toInt();
    READRESP = true;
  }
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  int packetSize = 242;  // Maximum payload size per packet

  // Calculate the number of packets needed
  int numPackets = (outgoing.length() + packetSize - 1) / packetSize;

  //Serial.println(numPackets);

  // if ( numPackets > 1 ) {
  //   LoRa.beginPacket(); // start packet
  //   LoRa.write(otherNode);      // add destination address
  //   LoRa.write(MasterNode);     // add sender address
  //   LoRa.write(msgCount);       // add message ID
  //   LoRa.write(String(numPackets).length());            // add payload length
  //   LoRa.print(String(numPackets)); // Add payload chunk
  //   LoRa.endPacket();           // finish packet and send it
  //   LoRa.receive();
  //   delay(500);
  // }

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
    delay(2000);
  }

  if (numPackets > 0) {
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

// Function to get sensor vals
void getReadings() {
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  lux = lightMeter.readLightLevel();
  soilMoistureValue = analogRead(soilmoisturePin);
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  rainfall = map(analogRead(rain_sensor), 1500, 4095, 100, 0);
  if (rainfall >= 100) {
    rainfall = 100;
  }
  if (rainfall <= 0) {
    rainfall = 0;
  }
  val1 = 0;
  val2 = 0;
  val3 = 0;
}
// Function to get time stamp
void getTimeStamp() {
  dayStamp = String(2024) + "-" + String(07) + "-" + String(23);
  timeStamp = String(hour) + ":" + String(minute) + ":" + String(second);
  minute = minute + 5;
  second = second + 4;
  neogps.available();

  for (int k = 0; k < 1500; k++) {
    //while (neogps.available() > 0) {
    gps.encode(neogps.read());
    //Serial.print("Latitude= ");
    Latitude = gps.location.lat();
    //Serial.print(Latitude, 6);
    //Serial.print(" Longitude= ");
    Longitude = gps.location.lng();
    //Serial.println(gps.location.lng(), 6);
    dayStamp = gps.date.value();
    //Serial.print("Raw date DDMMYY = ");
    //if (gps.time.isValid() || k == 1) {
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();
    hour = gps.time.hour() + 2;  //adjust UTC
    minute = gps.time.minute();
    second = gps.time.second();
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
    timeStamp = String(hour) + ":" + String(minute) + ":" + String(second);
    //Serial.println(timeStamp);
    //Serial.println();
    //}
    //}
  }
  neogps.end();
}

// Write the sensor readings on the SD card
void logSDCard() {
  dataMessage = "{Node" + String(device_id) + "; " + String(readingID) + "; " + String(dayStamp) + "; " + String(timeStamp) + "; " + String(temperature) + "; " + String(pressure) + "; " + String(humidity) + "; " + String(lux) + "; " + String(soilmoisturepercent) + "}\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
  // Add sensor data to buffer for LoRa transmission
  strcat(sensorDataBuffer, dataMessage.c_str());
}
// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char *path, const char *message) {
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
}
