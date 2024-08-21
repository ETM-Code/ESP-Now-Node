#include <WiFi.h>
#include <esp_now.h>
#include <cstdint>
#include <array>
#include <string.h>
#include "LSM6DSOXSensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <cmath>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"

esp_err_t esp_task_wdt_init(uint32_t timeout, bool panic);

LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

// Constants

// Define the LED pins
#define LED_PIN1 5
#define LED_PIN2 6
#define CLIENT_TIMEOUT 5

// Define the I2C pins
#define SCL_PIN 1
#define SDA_PIN 0

// Define ADXL375 SPI pins
#define ADXL375_SCK 13
#define ADXL375_MISO 12
#define ADXL375_MOSI 11
#define ADXL375_CS 10

// Define some random nonsense that I'm too tired to explain

#define SCAN_DURATION 4000
#define NODE_TRANSMISSION_INTERVAL_MIN 5000
#define NODE_TRANSMISSION_INTERVAL_MAX 10000

#define DATA_SIZE 6400
#define ACCEL_DATA_SIZE 3200
#define GYRO_DATA_SIZE 6670
#define TRANSMISSION_THRESHOLD 1
#define NUMBER_OF_SENSORS 6
#define TRANSMISSION_SIZE 250 //Maximum bytes transmissable by ESP_Now in a single batch
#define USABLE_TRANSMISSION_SPACE 234 //250-8-4-4=234

int divideAndRoundUp(int numerator, int denominator) {
    if (denominator == 0) {
        Serial.println("Denominator cannot be zero.");
    }
    // Perform integer division
    int result = numerator / denominator;
    // Check if there is a remainder
    if (numerator % denominator != 0) {
        result += 1; // Increment result if there is any remainder
    }
    return result;
}
const int numberOfTransmissions = divideAndRoundUp(DATA_SIZE, USABLE_TRANSMISSION_SPACE);
unsigned long lastTimeTime = 0;

#define SETUP_COMMAND 2069783108202043734ULL
#define FIRST_BITS 483868772U
#define LAST_BITS 2952687110U
#define WIFI_CHANNEL 1
#define SEND_INTERVAL 1000
#define MAX_WS_CLIENTS 40

#define MESSAGE_TAGS_TO_STORE 30
#define NO_OF_BATCH_TAGS 25

// Global Variables
bool setupReceived = false;

bool scanMode = false;
unsigned long scanEndTime = 0;

 uint8_t AP_Address[6];

uint32_t setupMessageFirst32Bits;
uint32_t setupMessageSecond32Bits;

AsyncWebSocket ws("/ws");
AsyncWebSocketClient * wsClient = nullptr;

typedef struct {
     uint8_t address[6];
} MacAddress;

 MacAddress macAddresses[15] = {0};
 uint8_t numSavedAddresses = 0;
 uint8_t macArrayNumber = 0;

 uint8_t messageAddresses[15][NO_OF_BATCH_TAGS][6];
 uint8_t messageData[15][NO_OF_BATCH_TAGS][250];
 uint32_t messageTags[MESSAGE_TAGS_TO_STORE] = {0};
 uint8_t messageTagBookmark = 0;
 uint8_t batchTags[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS] = {0};
 uint8_t batchTagsBookmark[MESSAGE_TAGS_TO_STORE] = {0};
 uint8_t batchData[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS][USABLE_TRANSMISSION_SPACE] = {0};
 uint8_t batchDataBookmark = 0;
 uint8_t currentPersonalBatchTag = 0;
 MacAddress myMacAddress;
//  uint8_t responseBuffer[DATA_SIZE+sizeof(MacAddress)+sizeof(bool)/*+sizeof(uint32_t)*/+1000];

AsyncWebServer server(80);
 std::vector<std::pair<AsyncWebSocketClient *, unsigned long>> clients;


float prevAccelX = 0;

bool breakLoops = false;

//  uint8_t buffer[6+4+1+USABLE_TRANSMISSION_SPACE] = {0};

unsigned long lastDataSend = 0;
 uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Defining Message Types
typedef struct {
    alignas(4) MacAddress macAddress;
    alignas(4) uint32_t messageTag;
    alignas(4) uint8_t batchTag;
    alignas(4) uint8_t data[USABLE_TRANSMISSION_SPACE];
} opTransmission;
const size_t opTransmissionSize = sizeof(opTransmission);

typedef struct {
    uint32_t code;
     uint8_t routerAddress[6];
} scanTransmission;
const size_t scanTransmissionSize = sizeof(scanTransmission);

unsigned long lastScanBroadcastTime = 0;

//  uint8_t data[DATA_SIZE] = {0}; //declared for send-random-data function to overcome stack allocation issues




typedef struct {
    alignas(4) MacAddress macAddress;
    alignas(4) uint32_t messageTag;
    alignas(4) uint8_t batchTag;
    uint8_t accelData [USABLE_TRANSMISSION_SPACE/2];
    uint8_t gyroData [USABLE_TRANSMISSION_SPACE/2];
} sensorTransmission;


//Sensor Data
// uint8_t accelDataFull [ACCEL_DATA_SIZE];
float accelDataFull [ACCEL_DATA_SIZE];
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
float gyroDataFull [GYRO_DATA_SIZE];
float gyroDataSquashed [ACCEL_DATA_SIZE];
sensorTransmission dataStore = {0};
// bool shouldTransmit = true;
// ^ Sensor Data

int sensorDataBookmark = 0;
int gyroDataBookmark = 0;
int accelsDataBookmark = 0;

void startScanMode();
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len);
// void sendRandomData();
bool addPeer(const uint8_t *);
// void generateRandomData(uint8_t* data, size_t length);
// void sendMessageToPeers(const uint8_t *message, size_t len);
bool isDuplicateMessage(uint32_t messageTag, uint8_t batchTag);
bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum);
bool isDuplicateAddress(const uint8_t* mac_addr);
void printMacAddress(const uint8_t* mac_addr);
void addInitialPeers();
void gatherSensorData();
void sendSensorData();

// WiFi Credentials
const char* ssid = "ESP32_Network";
const char* password = "password123";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

const int N = 6;  // Order of the filter

// Filter coefficients
const float b[N + 1] = {0.70397933f, 4.22387596f, 10.5596899f, 14.07958653f, 10.5596899f, 4.22387596f, 0.70397933f};
const float a[N + 1] = {1.0f, 5.40400985f, 12.21077247f, 14.76856606f, 10.08630847f, 3.68957553f, 0.56510196f};


// Function to apply the Chebyshev Type I filter
void applyChebyshevFilter(const float input[GYRO_DATA_SIZE], uint8_t filtered_output[ACCEL_DATA_SIZE]) {
    float x[N + 1] = {};  // Past input values
    float y[N + 1] = {};  // Past output values

    for (int n = 0; n < ACCEL_DATA_SIZE; ++n) {
        // Shift the values in x and y arrays
        for (int i = N; i > 0; --i) {
            x[i] = x[i - 1];
            y[i] = y[i - 1];
        }

        // Assign the current input value
        x[0] = input[n];

        // Calculate the current output value using the difference equation
        float output_value = b[0] * x[0];
        for (int i = 1; i <= N; ++i) {
            output_value += b[i] * x[i] - a[i] * y[i];
        }
        y[0] = output_value;

        // Store the output value
        filtered_output[n] = y[0];
    }
}

// Function to downsample the filtered output to 3200 elements
void downsample(const float filtered_input[GYRO_DATA_SIZE], float output[ACCEL_DATA_SIZE]) {
    const float factor = static_cast<float>(GYRO_DATA_SIZE) / ACCEL_DATA_SIZE;
    for (int i = 0; i < ACCEL_DATA_SIZE; ++i) {
        output[i] = filtered_input[static_cast<int>(i * factor)];
    }
}


void stringToMacAddress(String macStr, MacAddress* mac) {
    for (int i = 0; i < 6; ++i) {
        mac->address[i] = strtol(macStr.substring(i * 3, i * 3 + 2).c_str(), NULL, 16);
    }
}

void printPeerInfo() {
    esp_now_peer_num_t peerCount;
    if (esp_now_get_peer_num(&peerCount) == ESP_OK) {
        Serial.printf("Number of ESP-NOW peers: %d\n", peerCount.total_num);
        if (peerCount.total_num > 0) {
            esp_now_peer_info_t peerInfo;
            for (int i = 0; i < peerCount.total_num; ++i) {
                if (esp_now_fetch_peer(true, &peerInfo) == ESP_OK) {
                    Serial.printf("Peer %d: MAC: ", i + 1);
                    for (int j = 0; j < 6; j++) {
                        Serial.printf("%02X", peerInfo.peer_addr[j]);
                        if (j < 5) Serial.print(":");
                    }
                    Serial.printf(", Channel: %d, Encrypt: %d\n", peerInfo.channel, peerInfo.encrypt);
                } else {
                    Serial.printf("Failed to fetch details for peer %d\n", i + 1);
                }
            }
        }
    } else {
        Serial.println("Failed to get the number of ESP-NOW peers.");
    }
}

void removeAllPeers() {
    esp_now_deinit(); // Deinitialize ESP-NOW
    if (esp_now_init() != ESP_OK) { // Reinitialize ESP-NOW
        Serial.println("Error reinitializing ESP-NOW");
    }
    esp_now_register_recv_cb(onDataRecv);
    addInitialPeers(); // Re-add initial peers after reinitializing
}

bool checkForSpikes(){
    // Serial.println("Checking for spikes...");
    for(int i = 0; i<ACCEL_DATA_SIZE; i++){
        if(accelDataFull[i]>=TRANSMISSION_THRESHOLD){
            return true;
        }
    }
    return false;
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);

void initWebSocket() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
}


void setup() {
    esp_task_wdt_init(100, true);
    Serial.begin(115200);
    delay(1000);
    WiFi.mode(WIFI_AP_STA);
    Serial.println(WiFi.macAddress());
    Wire.begin(SDA_PIN, SCL_PIN);

    if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
        Serial.println("AP Config Failed");
    }
    WiFi.softAP(ssid, password);
    WiFi.setSleep(false);
    // Default clock is 100kHz. LSM6DSOX also supports 400kHz, let's use it
    Wire.setClock(100000);
    
    // Init the sensor
    lsm6dsoxSensor.begin();

    // Enable accelerometer and gyroscope, and check success
    if (lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
        Serial.println("Success enabling gyro");
    } else {
        Serial.println("Error enabling gyro");
    }

    // Read ID of device and check that it is correct
    uint8_t id;
    lsm6dsoxSensor.ReadID(&id);
    if (id != LSM6DSOX_ID) {
        Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
    } else {
        Serial.println("Receviced correct ID for LSM6DSOX sensor");
    }

    // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
    lsm6dsoxSensor.Set_G_FS(2000);

    // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
    lsm6dsoxSensor.Set_G_ODR(6667.0f);
    if(!accel.begin())
    {
        /* There was a problem detecting the ADXL375 ... check your connections */
        Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
        while(1);
    }

    adxl3xx_dataRate_t accelDataRate = ADXL343_DATARATE_3200_HZ; 
    accel.setDataRate(accelDataRate);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    
    initWebSocket();
    server.begin();

    addInitialPeers();

    stringToMacAddress(WiFi.macAddress(), &myMacAddress);
    Serial.println("Setup Complete");
}

void addInitialPeers() {
    // // esp_now_peer_info_t peerInfo = {};

    // // Add the broadcast peer
    // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    // peerInfo.channel = WIFI_CHANNEL;
    // peerInfo.encrypt = false;

    // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //     Serial.println("Failed to add broadcast peer");
    //     return;
    // }

    // Serial.println("Broadcast peer added");
}



void loop() {
    unsigned long currentMillis = millis(); //Used often, so defining is good
    gatherSensorData();


    if (millis() - lastTimeTime > 31) {
    lastTimeTime = millis();
    if (wsClient && wsClient->canSend()) {
        sendSensorData();
    }
}
    if (!scanMode && setupReceived) { //Switch to generate random data and add to buffer
        gatherSensorData();
        esp_now_del_peer(broadcastAddress);
        // if (checkForSpikes()) {
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println("Should be sending...");
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     // Serial.println();
        //     Serial.print("Accelerations:  ");
        //     for(int i=0; i<sizeof(accelDataFull-1); i++){
        //         Serial.print(accelDataFull[i]);
        //         Serial.print(", ");
        //     }
        //     Serial.println();
        //     Serial.print("Rotations:  ");
        //     for(int i=0; i<sizeof(accelDataFull-1); i++){
        //         Serial.print(gyroDataSquashed[i]);
        //         Serial.print(", ");
        //     }
        //     Serial.println();
        // }
    }

    // If in scan mode, broadcast scan messages periodically
    if (scanMode) {
        if (currentMillis - lastScanBroadcastTime > 1000) {
            esp_now_peer_info_t peerInfo = {};


            if (!esp_now_is_peer_exist(broadcastAddress)){
                // Add the broadcast peer
                memcpy(peerInfo.peer_addr, broadcastAddress, 6);
                peerInfo.channel = WIFI_CHANNEL;
                peerInfo.encrypt = false;

                if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                    Serial.println("Failed to add broadcast peer");
                    return;
                }
            }
            scanTransmission scanMessage;
            scanMessage.code = setupMessageFirst32Bits;
            memcpy(scanMessage.routerAddress, AP_Address, 6);

            Serial.println("Broadcasting scan message");

            esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&scanMessage, sizeof(scanMessage));
            if (result == ESP_OK) {
            } else {
                Serial.print("Error sending message: ");
                Serial.println(result);
            }
            lastScanBroadcastTime = currentMillis;
        }
    }

}


void gatherSensorData() {
    uint8_t gyroStatus;
    lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
    if (gyroStatus == 1) { // Status == 1 means a new data is available
        int32_t tempGyroData[3];  // Temporary array to store the gyroscope data as int32_t

        // Retrieve the gyroscope data from the LSM6DSOX sensor
        lsm6dsoxSensor.Get_G_Axes(tempGyroData);

        for(int i=0; i<3; i++){
            tempGyroData[i] = tempGyroData[i] * (PI / 180.0);
        }
    
        int32_t tempGyroDataAbs = (sqrt(tempGyroData[1]*tempGyroData[1]+tempGyroData[2]*tempGyroData[2]+tempGyroData[3]*tempGyroData[3]));
        }
    gyroDataFull[gyroDataBookmark];
    gyroDataBookmark = (gyroDataBookmark) % sizeof(accelDataFull)-1;
    // Serial.println();
    sensors_event_t event;
    accel.getEvent(&event);
    float aX = event.acceleration.x;
    if(aX!=prevAccelX){
        // Serial.print("New accelerometer data:  ");
        float aY = event.acceleration.y;
        float aZ = event.acceleration.z;
        accelDataFull[accelsDataBookmark] = sqrt(aX*aX+aY*aY+aZ*aZ);
        // Serial.println(accelDataFull[sensorDataBookmark]);
        accelsDataBookmark = (accelsDataBookmark) % sizeof(accelDataFull)-1;
    }

    // Serial.println();
}


unsigned long timeOfLastTransmission = 0;



void printMacAddress(const uint8_t* mac_addr) {
    for (int i = 0; i < 6; i++) {
        if (mac_addr[i] < 16) {
            Serial.print("0");
        }
        Serial.print(mac_addr[i], HEX);
        if (i < 5) {
            Serial.print(":");
        }
    }
    Serial.println();
}


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        if (clients.size() >= MAX_WS_CLIENTS) {
            client->close();}
            else {
            clients.push_back({client, millis()});}

        Serial.println("WebSocket client connected");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket client disconnected");
        clients.erase(std::remove_if(clients.begin(), clients.end(),
                                     [client](const std::pair<AsyncWebSocketClient *, unsigned long> &p) {
                                         return p.first == client;
                                     }),
                      clients.end());
    } else if (type == WS_EVT_DATA) {
    Serial.println("WebSocket data received");
    String msg = (char*)data;
    Serial.print("Message: ");
    Serial.println(msg);
    
    
    if (msg.indexOf("getData") != -1) {  // Check if "getData" is contained within msg
        Serial.println("Got request for data");
        
            // if(breakLoops){breakLoops = false; break;}

                // if(breakLoops){breakLoops = false; break;}
                    Serial.println("We're checking to send data");
                    // if(checkForSpikes()){
                        Serial.println("We're going to send data");

                        String sendingString = "";
                        Serial.println("Initialised sendingString");
                        for(int i=0; i<sizeof(MacAddress); i++){
                            sendingString+=String(myMacAddress.address[i]);
                            sendingString+=",";
                        }
                        Serial.println("Saved mac address");
                        for(int i=0; i<ACCEL_DATA_SIZE; i++){
                            sendingString+=String(accelDataFull[i]);
                            // sendingString+=",";
                            // Serial.print("Saved ");
                            // Serial.print(i);
                            // Serial.println("th accel");
                        }    
                        Serial.println("Saved accels");
                        downsample(gyroDataFull, gyroDataSquashed);
                        for(int i=0; i<ACCEL_DATA_SIZE; i++){
                            sendingString+=String(gyroDataSquashed[i]);
                            sendingString+=",";
                        }
                        Serial.println("Saved Gyros");
                

                        // uint32_t checksum = CRC32.crc32((uint8_t*)&messageData[addressIndex][messageIndex], dataSize);
                        // memcpy(responseBuffer + dataSize, &checksum, sizeof(checksum));
                        // Serial.print("Data to send:  ");
                        // Serial.println(sendingString);
                        client->text(sendingString);
                        sendingString = "";
                        Serial.println("Sent data");
                    // }
                
            }
    }
}

void sendSensorData() {
    if (wsClient && wsClient->canSend()) {
       Serial.println("Got request for data");
        // if(millis()-lastTimeTime>5000){
        //     lastTimeTime=millis();
            // if(breakLoops){breakLoops = false; break;}

                // if(breakLoops){breakLoops = false; break;}
                    Serial.println("We're checking to send data");
                    // if(checkForSpikes()){
                        Serial.println("We're going to send data");

                        String sendingString = "";
                        Serial.println("Initialised sendingString");
                        for(int i=0; i<sizeof(MacAddress); i++){
                            sendingString+=String(myMacAddress.address[i]);
                            sendingString+=",";
                        }
                        Serial.println("Saved mac address");
                        for(int i=0; i<ACCEL_DATA_SIZE; i++){
                            sendingString+=String(accelDataFull[i]);
                            // sendingString+=",";
                            // Serial.print("Saved ");
                            // Serial.print(i);
                            // Serial.println("th accel");
                        }    
                        Serial.println("Saved accels");
                        downsample(gyroDataFull, gyroDataSquashed);
                        for(int i=0; i<ACCEL_DATA_SIZE; i++){
                            sendingString+=String(gyroDataSquashed[i]);
                            sendingString+=",";
                        }
                        Serial.println("Saved Gyros");
                

                        // uint32_t checksum = CRC32.crc32((uint8_t*)&messageData[addressIndex][messageIndex], dataSize);
                        // memcpy(responseBuffer + dataSize, &checksum, sizeof(checksum));
                        // Serial.print("Data to send:  ");
                        // Serial.println(sendingString);
                        wsClient->text(sendingString);
                        sendingString = "";
                        Serial.println("Sent data");
        // }
    }
}