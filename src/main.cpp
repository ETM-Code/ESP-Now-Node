#include <WiFi.h>
#include <esp_now.h>
#include <cstdint>
#include <array>
#include <string.h>

// Constants
#define SCAN_DURATION 10000
#define NODE_TRANSMISSION_INTERVAL_MIN 5000
#define NODE_TRANSMISSION_INTERVAL_MAX 10000

#define DATA_SIZE 6000
#define TRANSMISSION_SIZE 250 //Maximum bytes transmissable by ESP_Now in a single batch
#define USABLE_TRANSMISSION_SPACE 239 //Allocate 4 bytes for message identifier, 1 for batch identifier, 6 for mac address

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

#define SETUP_COMMAND 2069783108202043734ULL
#define FIRST_BITS 483868772U
#define LAST_BITS 2952687110U
#define WIFI_CHANNEL 1
#define SEND_INTERVAL 1000

#define MESSAGE_TAGS_TO_STORE 30
#define NO_OF_BATCH_TAGS 25

// Global Variables
bool setupReceived = false;

bool scanMode = false;
unsigned long scanEndTime = 0;

uint8_t AP_Address[6];

uint32_t setupMessageFirst32Bits;
uint32_t setupMessageSecond32Bits;

typedef struct {
    uint8_t address [6];
} macAddress;

macAddress macAddresses[15] = {0};
uint8_t numSavedAddresses = 0;
uint8_t macArrayNumber = 0;

uint32_t messageTags[MESSAGE_TAGS_TO_STORE] = {0}; //Permits the identification of 45 different full transmissions -> MAKE SURE TO ALLOCATE [0] TO THIS ESP
uint8_t messageTagBookmark = 0;
uint8_t batchTags[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS] = {0}; //Identifies the part of a message that the batch contains, 24 batches per transmission
uint8_t batchTagsBookmark[MESSAGE_TAGS_TO_STORE] = {0};
uint8_t batchData[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS][USABLE_TRANSMISSION_SPACE] = {0};
uint8_t batchDataBookmark = 0;
uint8_t currentPersonalBatchTag = 0;
macAddress myMacAddress;

unsigned long lastDataSend = 0;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Defining Message Types
typedef struct {
    macAddress macAddress;
    uint32_t messageTag;
    uint8_t batchTag;
    uint8_t data[USABLE_TRANSMISSION_SPACE];
} opTransmission;
const size_t opTransmissionSize = sizeof(opTransmission);

typedef struct {
    uint32_t code;
    uint8_t routerAddress[6];
} scanTransmission;
const size_t scanTransmissionSize = sizeof(scanTransmission);

unsigned long lastScanBroadcastTime = 0;

void startScanMode();
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len);
void sendRandomData();
bool addPeer(const uint8_t *);
void generateRandomData(uint8_t* data, size_t length);
void sendMessageToPeers(const uint8_t *message, size_t len);
bool isDuplicateMessage(uint32_t messageTag, uint8_t batchTag);
bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum);
bool isDuplicateAddress(const uint8_t* mac_addr);
void printMacAddress(const uint8_t* mac_addr);

void stringToMacAddress(String macStr, macAddress* mac) {
    for (int i = 0; i < 6; ++i) {
        mac->address[i] = strtol(macStr.substring(i * 3, i * 3 + 2).c_str(), NULL, 16);
    }
}

bool sendESPNowMessage(const uint8_t *peer_addr, const uint8_t *data, size_t len) {
    esp_err_t result = esp_now_send(peer_addr, data, len);
    if (result == ESP_OK) {
        Serial.println("Message sent successfully");
        return true;
    } else {
        Serial.print("Error sending message: ");
        Serial.println(result);
        return false;
    }
}

void removeAllPeers() {
    esp_now_deinit(); // Deinitialize ESP-NOW
    if (esp_now_init() != ESP_OK) { // Reinitialize ESP-NOW
        Serial.println("Error reinitializing ESP-NOW");
    }
}

void setup() {
    Serial.begin(115200);
    delay(5000);
    WiFi.mode(WIFI_AP_STA);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    Serial.println("Setup complete, waiting for setup message...");
    // Add the broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        return;
    }

    Serial.println("ESP-NOW Initialized and Broadcast Peer Added");
    
    stringToMacAddress(WiFi.macAddress(), &myMacAddress);
}

void loop() {
    unsigned long currentMillis = millis(); //Used often, so defining is good

    if (scanMode && currentMillis > scanEndTime) { //Good
        scanMode = false;
        Serial.println("Scan mode ended.");
    }

    if (!scanMode && setupReceived) { //Switch to generate random data and add to buffer
        if (currentMillis - lastDataSend >= NODE_TRANSMISSION_INTERVAL_MIN + (esp_random() % (NODE_TRANSMISSION_INTERVAL_MAX - NODE_TRANSMISSION_INTERVAL_MIN))) {
            sendRandomData();
            lastDataSend = currentMillis;
        }
    }

    // If in scan mode, broadcast scan messages periodically
    if (scanMode) {
        if (currentMillis - lastScanBroadcastTime > 1000) {
            scanTransmission scanMessage;
            scanMessage.code = setupMessageFirst32Bits;
            memcpy(scanMessage.routerAddress, AP_Address, 6);

            Serial.println("Broadcasting scan message");

            sendESPNowMessage(broadcastAddress, (const uint8_t *)&scanMessage, scanTransmissionSize);
            lastScanBroadcastTime = currentMillis;
        }
    }
}

void startScanMode() {
    scanEndTime = millis() + SCAN_DURATION;
    Serial.println("Starting scan mode...");
    scanMode = true;
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.channel(1);  // You might need to scan multiple channels
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    Serial.print("Data received from ");
    printMacAddress(mac_addr);
    Serial.print(", length: ");
    Serial.println(len);
    uint64_t setupMessage;
    memcpy(&setupMessage, data, sizeof(uint64_t));

    if (len == sizeof(uint64_t) && setupMessage == SETUP_COMMAND) { //Check for setup message
        Serial.println("AP Setup Message Received");
        setupMessageFirst32Bits = (uint32_t)(setupMessage >> 32);
        setupMessageSecond32Bits = (uint32_t)(setupMessage & 0xFFFFFFFF);
        memcpy(AP_Address, mac_addr, 6);
        setupReceived = true;
        startScanMode();
        macArrayNumber = 0;
        numSavedAddresses = 0;

        removeAllPeers();

        if(addPeer(mac_addr)){
            Serial.println("Added AP Address");
        }
        else{Serial.println("Failed to add AP Address");}
    } else if (scanMode && len == scanTransmissionSize) { //check for scan message
        uint32_t receivedCode;
        uint8_t receivedAP_Address[6];
        memcpy(&receivedCode, data, 4);
        memcpy(receivedAP_Address, data + 4, 6);

        if (receivedCode == setupMessageFirst32Bits && memcmp(receivedAP_Address, AP_Address, 6) == 0) { //check if 1st part of handshake
            Serial.println("Initial pairing message received and verified");

            scanTransmission responseMessage;
            responseMessage.code = setupMessageSecond32Bits;
            memcpy(responseMessage.routerAddress, &AP_Address, 6);

            sendESPNowMessage(mac_addr, (const uint8_t *)&responseMessage, sizeof(responseMessage));
            Serial.print("Sent response message: ");
            Serial.println();

            if (!isDuplicateAddress(mac_addr)) { //check if duplicate using above function
                if (numSavedAddresses < 15) {
                    if (addPeer(mac_addr)) {
                        Serial.println("Peer added successfully");
                        numSavedAddresses++;
                        memcpy((void *)&macAddresses[macArrayNumber], mac_addr, 6);
                        macArrayNumber++;
                        Serial.println("Added to mac address list");
                    }
                } else {
                    Serial.println("Maximum Peers Reached! New peer not added!");
                }
            }
        } else if (receivedCode == setupMessageSecond32Bits && memcmp(receivedAP_Address, AP_Address, 6) == 0) { //check if 2nd part of handshake
            Serial.println("Second pairing message received and verified");

            if (!isDuplicateAddress(mac_addr)) { //check if duplicate -> turn this into a function, since it's repeated
                if (numSavedAddresses < 15) {
                    if (addPeer(mac_addr)) {
                        Serial.println("Peer added successfully");
                        numSavedAddresses++;
                        memcpy((void *)&macAddresses[macArrayNumber], mac_addr, 6);
                        macArrayNumber++;
                        Serial.println("Added to mac address list");
                    }
                } else {
                    Serial.println("Maximum Peers Reached! New peer not added!");
                }
            }
        }
    } else if (len > 5 && len < 251) { //check if data transmission
        uint32_t receivedTag;
        memcpy(&receivedTag, data, 4);
        uint8_t receivedBatchTag;
        memcpy(&receivedBatchTag, data + 4, 1);
        Serial.printf("Mesh transmission received, tag: %u\n", receivedTag);

        if (!isDuplicateMessage(receivedTag, receivedBatchTag)) {
            if (messageTagBookmark >= MESSAGE_TAGS_TO_STORE) {
                messageTagBookmark = 0;
            }
            messageTags[messageTagBookmark] = receivedTag;
            sendMessageToPeers(data, len);
            sendESPNowMessage(AP_Address, (const uint8_t *)&len, len);
        } else {
            Serial.println("Duplicate message, discarding.");
        }
    }
}

void sendRandomData() {
    uint8_t data[DATA_SIZE];
    generateRandomData(data, DATA_SIZE);
    uint32_t messageIdentifier = esp_random();

    Serial.print("Sending random data to AP and peers: ");
    for (int i = 0; i < numberOfTransmissions; i++) {
        opTransmission message;
        message.macAddress = myMacAddress;
        message.messageTag = messageIdentifier;
        message.batchTag = i;
        if (i >= (numberOfTransmissions - 1)) {
            for (int j = 0; j < (DATA_SIZE % USABLE_TRANSMISSION_SPACE); j++) { //modify to handle cases where there's not a full batch, will be once per transmission, minimum
                message.data[j] = data[(i * USABLE_TRANSMISSION_SPACE) + j];
            }
        } else {
            for (int j = 0; j < USABLE_TRANSMISSION_SPACE; j++) { //modify to handle cases where there's not a full batch, will be once per transmission, minimum
                message.data[j] = data[(i * USABLE_TRANSMISSION_SPACE) + j];
            }
        }
        sendESPNowMessage(AP_Address, (const uint8_t *)&message, opTransmissionSize);
        sendMessageToPeers((const uint8_t *)&message, opTransmissionSize);
    }
}

void generateRandomData(uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        float rand_val = (float)esp_random() / UINT32_MAX;
        if (rand_val < 0.99) {
            data[i] = esp_random() % 4; // 0 to 3
        } else {
            data[i] = 10 + esp_random() % 141; // 10 to 150
        }
    }
}

void sendMessageToPeers(const uint8_t *message, size_t len) {
    Serial.println("Sending message to peers:");
    for (int i = 0; i < numSavedAddresses; i++) {
        Serial.print("Sending to peer: ");
        printMacAddress((const uint8_t *)&macAddresses[i]);
        sendESPNowMessage((const uint8_t *)&macAddresses[i], message, len);
    }
}

bool isDuplicateMessage(uint32_t messageTag, uint8_t batchTag) {
    for (int i = 0; i < MESSAGE_TAGS_TO_STORE; i++) {
        if (messageTag == messageTags[i]) {
            if (isDuplicateBatch(messageTag, batchTag, i)) {
                return true;
            } else {
                // If batch number is not found, add it
                batchTags[i][batchTagsBookmark[i]] = batchTag;
                batchTagsBookmark[i] = (batchTagsBookmark[i] + 1) % NO_OF_BATCH_TAGS; // Wrap around correctly
                return false;
            }
        }
    }
    // If message number is not found, add it
    messageTags[messageTagBookmark] = messageTag;
    batchTags[messageTagBookmark][0] = batchTag; // Initialize the first batchTag for the new messageTag
    batchTagsBookmark[messageTagBookmark] = 1;   // Initialize the batchTagsBookmark for the new messageTag
    messageTagBookmark = (messageTagBookmark + 1) % MESSAGE_TAGS_TO_STORE;

    return false;
}

bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum) {
    for (int i = 0; i < NO_OF_BATCH_TAGS; i++) {
        if (batchTag == batchTags[messageTagNum][i]) {
            return true;
        }
    }
    return false;
}

bool isDuplicateAddress(const uint8_t* mac_addr) {
    for (const auto& addr : macAddresses) {
        if (memcmp(addr.address, mac_addr, 6) == 0) {
            return true;
        }
    }
    return false;
}

bool addPeer(const uint8_t* mac_addr) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac_addr, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

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
