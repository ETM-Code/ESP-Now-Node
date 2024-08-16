// #include <WiFi.h>
// #include <esp_now.h>
// #include <cstdint>
// #include <array>
// #include <string.h>
// #include "LSM6DSOXSensor.h"
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL375.h>
// #include <cmath>

// LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

// // Constants

// // Define the LED pins
// #define LED_PIN1 5
// #define LED_PIN2 6

// // Define the I2C pins
// #define SCL_PIN 1
// #define SDA_PIN 0

// // Define ADXL375 SPI pins
// #define ADXL375_SCK 13
// #define ADXL375_MISO 12
// #define ADXL375_MOSI 11
// #define ADXL375_CS 10

// // Define some random nonsense that I'm too tired to explain

// #define SCAN_DURATION 4000
// #define NODE_TRANSMISSION_INTERVAL_MIN 5000
// #define NODE_TRANSMISSION_INTERVAL_MAX 10000

// #define DATA_SIZE 6400
// #define ACCEL_DATA_SIZE 3200
// #define GYRO_DATA_SIZE 6670
// #define TRANSMISSION_THRESHOLD 3
// #define NUMBER_OF_SENSORS 6
// #define TRANSMISSION_SIZE 250 //Maximum bytes transmissable by ESP_Now in a single batch
// #define USABLE_TRANSMISSION_SPACE 234 //250-8-4-4=234

// int divideAndRoundUp(int numerator, int denominator) {
//     if (denominator == 0) {
//         Serial.println("Denominator cannot be zero.");
//     }
//     // Perform integer division
//     int result = numerator / denominator;
//     // Check if there is a remainder
//     if (numerator % denominator != 0) {
//         result += 1; // Increment result if there is any remainder
//     }
//     return result;
// }
// const int numberOfTransmissions = divideAndRoundUp(DATA_SIZE, USABLE_TRANSMISSION_SPACE);

// #define SETUP_COMMAND 2069783108202043734ULL
// #define FIRST_BITS 483868772U
// #define LAST_BITS 2952687110U
// #define WIFI_CHANNEL 1
// #define SEND_INTERVAL 1000

// #define MESSAGE_TAGS_TO_STORE 30
// #define NO_OF_BATCH_TAGS 25

// // Global Variables
// bool setupReceived = false;

// bool scanMode = false;
// unsigned long scanEndTime = 0;

//  uint8_t AP_Address[6];

// uint32_t setupMessageFirst32Bits;
// uint32_t setupMessageSecond32Bits;

// typedef struct {
//      uint8_t address[6];
// } MacAddress;

//  MacAddress macAddresses[15] = {0};
//  uint8_t numSavedAddresses = 0;
//  uint8_t macArrayNumber = 0;

//  uint8_t messageAddresses[15][NO_OF_BATCH_TAGS][6];
//  uint8_t messageData[15][NO_OF_BATCH_TAGS][250];
//  uint32_t messageTags[MESSAGE_TAGS_TO_STORE] = {0};
//  uint8_t messageTagBookmark = 0;
//  uint8_t batchTags[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS] = {0};
//  uint8_t batchTagsBookmark[MESSAGE_TAGS_TO_STORE] = {0};
//  uint8_t batchData[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS][USABLE_TRANSMISSION_SPACE] = {0};
//  uint8_t batchDataBookmark = 0;
//  uint8_t currentPersonalBatchTag = 0;
//  MacAddress myMacAddress;

// float prevAccelX = 0;

// //  uint8_t buffer[6+4+1+USABLE_TRANSMISSION_SPACE] = {0};

// unsigned long lastDataSend = 0;
//  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// //Defining Message Types
// typedef struct {
//     alignas(4) MacAddress macAddress;
//     alignas(4) uint32_t messageTag;
//     alignas(4) uint8_t batchTag;
//     alignas(4) uint8_t data[USABLE_TRANSMISSION_SPACE];
// } opTransmission;
// const size_t opTransmissionSize = sizeof(opTransmission);

// typedef struct {
//     uint32_t code;
//      uint8_t routerAddress[6];
// } scanTransmission;
// const size_t scanTransmissionSize = sizeof(scanTransmission);

// unsigned long lastScanBroadcastTime = 0;

// //  uint8_t data[DATA_SIZE] = {0}; //declared for send-random-data function to overcome stack allocation issues




// typedef struct {
//     alignas(4) MacAddress macAddress;
//     alignas(4) uint32_t messageTag;
//     alignas(4) uint8_t batchTag;
//     uint8_t accelData [USABLE_TRANSMISSION_SPACE/2];
//     uint8_t gyroData [USABLE_TRANSMISSION_SPACE/2];
// } sensorTransmission;


// //Sensor Data
// uint8_t accelDataFull [ACCEL_DATA_SIZE];
// Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
// float gyroDataFull [GYRO_DATA_SIZE];
// uint8_t gyroDataSquashed [ACCEL_DATA_SIZE];
// sensorTransmission dataStore = {0};
// // bool shouldTransmit = true;
// // ^ Sensor Data

// int sensorDataBookmark = 0;

// void startScanMode();
// void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len);
// // void sendRandomData();
// bool addPeer(const uint8_t *);
// // void generateRandomData(uint8_t* data, size_t length);
// // void sendMessageToPeers(const uint8_t *message, size_t len);
// bool isDuplicateMessage(uint32_t messageTag, uint8_t batchTag);
// bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum);
// bool isDuplicateAddress(const uint8_t* mac_addr);
// void printMacAddress(const uint8_t* mac_addr);
// void addInitialPeers();
// void gatherSensorData();
// void sendSensorData();



// const int N = 6;  // Order of the filter

// // Filter coefficients
// const float b[N + 1] = {0.70397933f, 4.22387596f, 10.5596899f, 14.07958653f, 10.5596899f, 4.22387596f, 0.70397933f};
// const float a[N + 1] = {1.0f, 5.40400985f, 12.21077247f, 14.76856606f, 10.08630847f, 3.68957553f, 0.56510196f};


// // Function to apply the Chebyshev Type I filter
// void applyChebyshevFilter(const float input[GYRO_DATA_SIZE], uint8_t filtered_output[ACCEL_DATA_SIZE]) {
//     float x[N + 1] = {};  // Past input values
//     float y[N + 1] = {};  // Past output values

//     for (int n = 0; n < ACCEL_DATA_SIZE; ++n) {
//         // Shift the values in x and y arrays
//         for (int i = N; i > 0; --i) {
//             x[i] = x[i - 1];
//             y[i] = y[i - 1];
//         }

//         // Assign the current input value
//         x[0] = input[n];

//         // Calculate the current output value using the difference equation
//         float output_value = b[0] * x[0];
//         for (int i = 1; i <= N; ++i) {
//             output_value += b[i] * x[i] - a[i] * y[i];
//         }
//         y[0] = output_value;

//         // Store the output value
//         filtered_output[n] = y[0];
//     }
// }

// // Function to downsample the filtered output to 3200 elements
// void downsample(const float filtered_input[GYRO_DATA_SIZE], float output[ACCEL_DATA_SIZE]) {
//     const float factor = static_cast<float>(GYRO_DATA_SIZE) / ACCEL_DATA_SIZE;
//     for (int i = 0; i < ACCEL_DATA_SIZE; ++i) {
//         output[i] = filtered_input[static_cast<int>(i * factor)];
//     }
// }


// void stringToMacAddress(String macStr, MacAddress* mac) {
//     for (int i = 0; i < 6; ++i) {
//         mac->address[i] = strtol(macStr.substring(i * 3, i * 3 + 2).c_str(), NULL, 16);
//     }
// }

// bool sendESPNowMessage(const uint8_t *peer_addr, const uint8_t *data, size_t len) {
//     esp_err_t result = esp_now_send(peer_addr, data, sizeof(data));
//     if (result == ESP_OK) {
//         Serial.println("Message sent successfully");
//         return true;
//     } else {
//         Serial.print("Error sending message: ");
//         Serial.println(result);
//         return false;
//     }
// }

// void printPeerInfo() {
//     esp_now_peer_num_t peerCount;
//     if (esp_now_get_peer_num(&peerCount) == ESP_OK) {
//         Serial.printf("Number of ESP-NOW peers: %d\n", peerCount.total_num);
//         if (peerCount.total_num > 0) {
//             esp_now_peer_info_t peerInfo;
//             for (int i = 0; i < peerCount.total_num; ++i) {
//                 if (esp_now_fetch_peer(true, &peerInfo) == ESP_OK) {
//                     Serial.printf("Peer %d: MAC: ", i + 1);
//                     for (int j = 0; j < 6; j++) {
//                         Serial.printf("%02X", peerInfo.peer_addr[j]);
//                         if (j < 5) Serial.print(":");
//                     }
//                     Serial.printf(", Channel: %d, Encrypt: %d\n", peerInfo.channel, peerInfo.encrypt);
//                 } else {
//                     Serial.printf("Failed to fetch details for peer %d\n", i + 1);
//                 }
//             }
//         }
//     } else {
//         Serial.println("Failed to get the number of ESP-NOW peers.");
//     }
// }

// void removeAllPeers() {
//     esp_now_deinit(); // Deinitialize ESP-NOW
//     if (esp_now_init() != ESP_OK) { // Reinitialize ESP-NOW
//         Serial.println("Error reinitializing ESP-NOW");
//     }
//     esp_now_register_recv_cb(onDataRecv);
//     addInitialPeers(); // Re-add initial peers after reinitializing
// }

// bool checkForSpikes(){
//     // Serial.println("Checking for spikes...");
//     for(int i = 0; i<ACCEL_DATA_SIZE; i++){
//         if(accelDataFull[i]>=TRANSMISSION_THRESHOLD){
//             return true;
//         }
//     }
//     return false;
// }

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
//     WiFi.mode(WIFI_AP_STA);

//     Wire.begin(SDA_PIN, SCL_PIN);

//     // Default clock is 100kHz. LSM6DSOX also supports 400kHz, let's use it
//     Wire.setClock(100000);

//     // Init the sensor
//     lsm6dsoxSensor.begin();

//     // Enable accelerometer and gyroscope, and check success
//     if (lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
//         Serial.println("Success enabling gyro");
//     } else {
//         Serial.println("Error enabling gyro");
//     }

//     // Read ID of device and check that it is correct
//     uint8_t id;
//     lsm6dsoxSensor.ReadID(&id);
//     if (id != LSM6DSOX_ID) {
//         Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
//     } else {
//         Serial.println("Receviced correct ID for LSM6DSOX sensor");
//     }

//     // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
//     lsm6dsoxSensor.Set_G_FS(2000);

//     // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
//     lsm6dsoxSensor.Set_G_ODR(6667.0f);
//     if(!accel.begin())
//     {
//         /* There was a problem detecting the ADXL375 ... check your connections */
//         Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
//         while(1);
//     }

//     adxl3xx_dataRate_t accelDataRate = ADXL343_DATARATE_3200_HZ; 
//     accel.setDataRate(accelDataRate);
    
//     // Initialize ESP-NOW
//     if (esp_now_init() != ESP_OK) {
//         Serial.println("Error initializing ESP-NOW");
//         return;
//     }

//     esp_now_register_recv_cb(onDataRecv);

//     Serial.println("Setup complete, waiting for setup message...");
//     addInitialPeers();

//     stringToMacAddress(WiFi.macAddress(), &myMacAddress);
// }

// void addInitialPeers() {
//     // // esp_now_peer_info_t peerInfo = {};

//     // // Add the broadcast peer
//     // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//     // peerInfo.channel = WIFI_CHANNEL;
//     // peerInfo.encrypt = false;

//     // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//     //     Serial.println("Failed to add broadcast peer");
//     //     return;
//     // }

//     // Serial.println("Broadcast peer added");
// }



// void loop() {
//     unsigned long currentMillis = millis(); //Used often, so defining is good

//     if (scanMode && currentMillis > scanEndTime) { //Good
//         scanMode = false;
//         Serial.println("Scan mode ended.");
//         printPeerInfo();
//     }

//     if (!scanMode && setupReceived) { //Switch to generate random data and add to buffer
//         gatherSensorData();
//         esp_now_del_peer(broadcastAddress);
//         if (checkForSpikes()) {
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println("Should be sending...");
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             // Serial.println();
//             sendSensorData();
//         }
//     }

//     // If in scan mode, broadcast scan messages periodically
//     if (scanMode) {
//         if (currentMillis - lastScanBroadcastTime > 1000) {
//             esp_now_peer_info_t peerInfo = {};


//             if (!esp_now_is_peer_exist(broadcastAddress)){
//                 // Add the broadcast peer
//                 memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//                 peerInfo.channel = WIFI_CHANNEL;
//                 peerInfo.encrypt = false;

//                 if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//                     Serial.println("Failed to add broadcast peer");
//                     return;
//                 }
//             }
//             scanTransmission scanMessage;
//             scanMessage.code = setupMessageFirst32Bits;
//             memcpy(scanMessage.routerAddress, AP_Address, 6);

//             Serial.println("Broadcasting scan message");

//             esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&scanMessage, sizeof(scanMessage));
//             if (result == ESP_OK) {
//             } else {
//                 Serial.print("Error sending message: ");
//                 Serial.println(result);
//             }
//             lastScanBroadcastTime = currentMillis;
//         }
//     }

// }

// void startScanMode() {
//     scanEndTime = millis() + SCAN_DURATION;
//     Serial.println("Starting scan mode...");
//     scanMode = true;
// }

// bool on = false;

// void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
//     Serial.println("Got some data");
//     if(on){
//     digitalWrite(LED_PIN2, LOW);
//     on = false;
//     }
//     else{digitalWrite(LED_PIN2, HIGH);
//     on = true;
//     }
//     uint64_t setupMessage;
//     memcpy(&setupMessage, data, sizeof(uint64_t));

//     if (len == sizeof(uint64_t) /*&& setupMessage == SETUP_COMMAND*/ && !scanMode) { //Check for setup message
//         Serial.println("AP Setup Message Received");
//         startScanMode();
//         setupMessageFirst32Bits = (uint32_t)(setupMessage >> 32);
//         setupMessageSecond32Bits = (uint32_t)(setupMessage & 0xFFFFFFFF);
//         memcpy(AP_Address, mac_addr, 6);
//         setupReceived = true;
//         macArrayNumber = 0;
//         numSavedAddresses = 0;

//         // removeAllPeers();

//         if(addPeer(mac_addr)){
//             // Serial.println("Added AP Address");
//             // Serial.print("Size of setup message: ");
//             // Serial.println(sizeof(setupMessage));
//             // Serial.print("Length of message: ");
//             // Serial.println(len);
//             // Send setup message back to AP
//             esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&setupMessage, len);
//             if (result == ESP_OK) {
//                 Serial.println("Setup message sent back to AP");
//             } else {
//                 Serial.println("Failed to send setup message back to AP");
//                 Serial.println(result);
//             }
//             // if (sendESPNowMessage(mac_addr, (const uint8_t*)&setupMessage, len)) {
                
//             // } else {
                
//             // }
//         }
//         else {
//             Serial.println("Failed to add AP Address");
//         }
//     } else if (scanMode && len == scanTransmissionSize) { //check for scan message
//         uint32_t receivedCode;
//         uint8_t receivedAP_Address[6];
//         memcpy(&receivedCode, data, 4);
//         memcpy(receivedAP_Address, data + 4, 6);

//         if (receivedCode == setupMessageFirst32Bits && memcmp(receivedAP_Address, AP_Address, 6) == 0) { //check if 1st part of handshake

//             scanTransmission responseMessage;
//             responseMessage.code = setupMessageSecond32Bits;
//             memcpy(responseMessage.routerAddress, &AP_Address, 6);

//             esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&responseMessage, sizeof(responseMessage));
//             if (result == ESP_OK) {
//                 Serial.println("Message sent successfully");
//             } else {
//                 Serial.print("Error sending message: ");
//                 Serial.println(result);
//             }
//             Serial.print("Sent response message: ");
//             Serial.println();

//             if (!isDuplicateAddress(mac_addr)) { //check if duplicate using above function
//                 if (numSavedAddresses < 15) {
//                     if (addPeer(mac_addr)) {
//                         Serial.println("Peer added successfully");
//                         Serial.print("Peer Mac Address: ");
//                         printMacAddress(mac_addr);
//                         numSavedAddresses++;
//                         memcpy((void *)&macAddresses[macArrayNumber], mac_addr, 6);
//                         macArrayNumber++;
//                         Serial.println("Added to mac address list");
//                     }
//                 } else {
//                     Serial.println("Maximum Peers Reached! New peer not added!");
//                 }
//             }
//         } else if (receivedCode == setupMessageSecond32Bits && memcmp(receivedAP_Address, AP_Address, 6) == 0) { //check if 2nd part of handshake

//             if (!isDuplicateAddress(mac_addr)) { //check if duplicate -> turn this into a function, since it's repeated
//                 if (numSavedAddresses < 15) {
//                     if (addPeer(mac_addr)) {
//                         Serial.println("Peer added successfully");
//                         Serial.print("Peer Mac Address: ");
//                         printMacAddress(mac_addr);
//                         numSavedAddresses++;
//                         memcpy((void *)&macAddresses[macArrayNumber], mac_addr, 6);
//                         macArrayNumber++;
//                         Serial.println("Added to mac address list");
//                     }
//                 } else {
//                     Serial.println("Maximum Peers Reached! New peer not added!");
//                 }
//             }
//         }
//     } 
//     // else if (len>=4){
//     //     uint32_t receivedCode;
//     //     memcpy(&receivedCode, data, 4);
//     //     Serial.print("Received code: ");
//     //     Serial.println(receivedCode);
//     // }
//     else if (len > 64 && len < 251 && !scanMode && esp_now_is_peer_exist(mac_addr)) { //check if data transmission
//         MacAddress incomingAddress;
//         for(int i = 0; i<6; i++){
//         memcpy(&incomingAddress.address[i], data+i, 1);}
//         uint32_t receivedTag;
//         memcpy(&receivedTag, data + 6, 4);
//         uint8_t receivedBatchTag;
//         memcpy(&receivedBatchTag, data + 10, 1);
//         Serial.printf("Mesh transmission received, tag: %u\n", receivedTag);

//         if (!isDuplicateMessage(receivedTag, receivedBatchTag)) {
//             if (messageTagBookmark >= MESSAGE_TAGS_TO_STORE) {
//                 messageTagBookmark = 0;
//             }
//             messageTags[messageTagBookmark] = receivedTag;
//             // sendMessageToPeers(data, len);
//             esp_now_send(0, data, len);
//             // sendESPNowMessage(AP_Address, (const uint8_t *)&len, len);
//             esp_now_send(AP_Address, data, len);
//         } else {
//             Serial.println("Duplicate message, discarding.");
//         }
//     }
//     // delay(50);
// }

// bool oneOn = false; 

// void gatherSensorData() {
//     uint8_t gyroStatus;
//     lsm6dsoxSensor.Get_G_DRDY_Status(&gyroStatus);
//     if (gyroStatus == 1) { // Status == 1 means a new data is available
//         int32_t tempGyroData[3];  // Temporary array to store the gyroscope data as int32_t

//         // Retrieve the gyroscope data from the LSM6DSOX sensor
//         lsm6dsoxSensor.Get_G_Axes(tempGyroData);

//         for(int i=0; i<3; i++){
//             tempGyroData[i] = tempGyroData[i] * (PI / 180.0);
//         }
    
//         int32_t tempGyroDataAbsSquashed = (sqrt(tempGyroData[1]*tempGyroData[1]+tempGyroData[2]*tempGyroData[2]+tempGyroData[3]*tempGyroData[3]))/100; //Squashes the gyro data into a form more convenient for uint8_t conversion
    

//         // Serial.println("Gyro values: ");
//         // Downcast and store the gyroscope data as uint8_t
//             gyroDataFull[sensorDataBookmark] = static_cast<uint8_t>(tempGyroDataAbsSquashed);
//             // Note: static_cast<uint8_t>(tempGyroData[i] & 0xFF) only stores the least significant byte of each int32_t value -> Make it into something better irl
//             // Serial.print(dataStore.gyroData[sensorDataBookmark][i]);
//             // Serial.print(", ");
//         }
    
//     // Serial.println();
//     sensors_event_t event;
//     accel.getEvent(&event);
//     float aX = event.acceleration.x/9.81;
//     if(aX!=prevAccelX){
//         // Serial.print("New accelerometer data:  ");
//         float aY = event.acceleration.y/9.81;
//         float aZ = event.acceleration.z/9.81;
//         accelDataFull[sensorDataBookmark] = (uint8_t)sqrt(aX*aX+aY*aY+aZ*aZ);
//         // Serial.println(accelDataFull[sensorDataBookmark]);
//     }


//     // Serial.println("Acceleration Values: ");
//     // for (int i = 0; i < 3; i++) {
//     //         Serial.print(dataStore.accelData[sensorDataBookmark][i]);
//     //         Serial.print(", ");
//     //     }
//     sensorDataBookmark = (sensorDataBookmark+1) % sizeof(dataStore.accelData);
//     // Serial.println();
// }


// unsigned long timeOfLastTransmission = 0;
// void sendSensorData(){
//     // if(millis()-timeOfLastTransmission>1000){
//         timeOfLastTransmission=millis();
//     Serial.println("Started sending function");
//     size_t bytesToCopy = sizeof(dataStore.accelData);  // 117 bytes
//     dataStore.messageTag = esp_random();
    
//     applyChebyshevFilter(gyroDataFull, gyroDataSquashed);
//     unsigned long timeOfLastBatch = 0;
//     for (int i = 0; i < numberOfTransmissions; i++) {
//         // while(millis()-timeOfLastBatch<500){}
//         // timeOfLastBatch=millis();
//         // Serial.println("looping");
//         // Serial.println("Entered first for loop");
//         dataStore.macAddress = myMacAddress;
//         dataStore.batchTag = i;

//         // memcpy(dataStore.accelData, accelDataFull + i * bytesToCopy, bytesToCopy);
//         // memcpy(dataStore.gyroData, gyroDataFull + i * bytesToCopy, bytesToCopy);

//         // Serial.println("Entered 2nd For loop");
//         if(i*(USABLE_TRANSMISSION_SPACE/2)<=sizeof(accelDataFull)){
//             // Serial.println("Where we're meant to be");
//             memcpy((void*)&dataStore.accelData, (void*)&accelDataFull[i * (USABLE_TRANSMISSION_SPACE/2)], (USABLE_TRANSMISSION_SPACE/2));
//             memcpy((void*)&dataStore.gyroData, (void*)&gyroDataSquashed[(i * (USABLE_TRANSMISSION_SPACE/2))+(USABLE_TRANSMISSION_SPACE/2)], (USABLE_TRANSMISSION_SPACE/2));
//         }
//         else{
//             // Serial.println("Not where we're meant to be");
//             size_t copySpace = sizeof(accelDataFull)-((i-1)*USABLE_TRANSMISSION_SPACE);
//             memcpy((void*)&dataStore.accelData, (void*)&accelDataFull[i * (USABLE_TRANSMISSION_SPACE/2)], copySpace/2);
//             memcpy((void*)&dataStore.gyroData, (void*)&gyroDataSquashed[(i * (USABLE_TRANSMISSION_SPACE/2))+(copySpace/2)], copySpace/2);
//         }
//         // Serial.println("Attempting to send");
//         Serial.println(dataStore.batchTag);
//         esp_now_send(0, (const uint8_t *)&dataStore, USABLE_TRANSMISSION_SPACE);
//         delay(10);
//         // uint8_t weirdArray[sizeof(dataStore)] = {0};
//         // memcpy(weirdArray, (const uint8_t *)&dataStore+11, sizeof(dataStore));
//         // Serial.print("Data we sent:  ");
//         // for(int i=0; i<USABLE_TRANSMISSION_SPACE; i++){
//         //     Serial.print(weirdArray[i]);
//         //     Serial.print(", ");
//         // }
//         // Serial.println();

//     }

//     Serial.println("Exited Loop");
//     memset(accelDataFull, 0, ACCEL_DATA_SIZE);
//     memset(gyroDataFull, 0, GYRO_DATA_SIZE);
//         if(oneOn){
//         digitalWrite(LED_PIN1, LOW);
//         oneOn = false;
//         }
//         else{digitalWrite(LED_PIN1, HIGH);
//         oneOn = true;
//         }
//     Serial.println("Finished with function");
//     // }
    
// }


// bool isDuplicateMessage(uint32_t messageTag, uint8_t batchTag) {
//     for (int i = 0; i < MESSAGE_TAGS_TO_STORE; i++) {
//         if (messageTag == messageTags[i]) {
//             if (isDuplicateBatch(messageTag, batchTag, i)) {
//                 return true;
//             } else {
//                 // If batch number is not found, add it
//                 batchTags[i][batchTagsBookmark[i]] = batchTag;
//                 batchTagsBookmark[i] = (batchTagsBookmark[i] + 1) % NO_OF_BATCH_TAGS; // Wrap around correctly
//                 return false;
//             }
//         }
//     }
//     // If message number is not found, add it
//     messageTags[messageTagBookmark] = messageTag;
//     batchTags[messageTagBookmark][0] = batchTag; // Initialize the first batchTag for the new messageTag
//     batchTagsBookmark[messageTagBookmark] = 1;   // Initialize the batchTagsBookmark for the new messageTag
//     messageTagBookmark = (messageTagBookmark + 1) % MESSAGE_TAGS_TO_STORE;

//     return false;
// }

// bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum) {
//     for (int i = 0; i < NO_OF_BATCH_TAGS; i++) {
//         if (batchTag == batchTags[messageTagNum][i]) {
//             return true;
//         }
//     }
//     return false;
// }

// bool isDuplicateAddress(const uint8_t* mac_addr) {
//     for (const auto& addr : macAddresses) {
//         if (memcmp(addr.address, mac_addr, 6) == 0) {
//             return true;
//         }
//     }
//     return false;
// }

// bool addPeer(const uint8_t* mac_addr) {
//     esp_now_del_peer(mac_addr);
//     esp_now_peer_info_t peerInfo = {};
//     memcpy(peerInfo.peer_addr, mac_addr, 6);
//     peerInfo.channel = WIFI_CHANNEL;
//     peerInfo.encrypt = false;
//     return esp_now_add_peer(&peerInfo) == ESP_OK;
// }

// void printMacAddress(const uint8_t* mac_addr) {
//     for (int i = 0; i < 6; i++) {
//         if (mac_addr[i] < 16) {
//             Serial.print("0");
//         }
//         Serial.print(mac_addr[i], HEX);
//         if (i < 5) {
//             Serial.print(":");
//         }
//     }
//     Serial.println();
// }
