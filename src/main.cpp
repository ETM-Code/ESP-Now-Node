#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <cstring>

// Definitions
#define MAX_PEERS 30
#define MAX_MESSAGES_PER_NODE 3
#define MAX_DATA_SIZE 6000
#define CHUNK_SIZE 250

#define SETUP_COMMAND 2069783108202043734ULL
#define FIRST_BITS 483868772U
#define LAST_BITS 2952687110U
#define MESH_TRANSMISSION_INT 12345678
#define ROUTER_TRANSMISSION_INT 87654321
#define SCAN_DURATION 10000
#define WIFI_CHANNEL 1
#define FIRST_N_BYTES_TO_PRINT 5
// #define MESSAGE_TAGS_TO_STORE 30
#define NO_OF_BATCH_TAGS 25
#define MAC_ADDRESSES_TO_STORE 5

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



#define WIFI_CHANNEL 1
#define SEND_INTERVAL 1000

#define MESSAGE_TAGS_TO_STORE 15
#define NO_OF_BATCH_TAGS 25
#define MESSAGES_TO_STORE 1

uint32_t router_transmission_int = ROUTER_TRANSMISSION_INT;

// WiFi Credentials
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char* ssid = "ESP32_Network";
const char* password = "password123";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

typedef struct {
    alignas(4) uint8_t address[6];
} MacAddress;

alignas(4) MacAddress macAddresses[MAC_ADDRESSES_TO_STORE] = {0};
alignas(4) uint8_t macAddressBookmark = 0;

size_t totalSize = (DATA_SIZE*MESSAGES_TO_STORE*MAC_ADDRESSES_TO_STORE)+(MAC_ADDRESSES_TO_STORE*8);
uint8_t responseBuffer[(DATA_SIZE*MESSAGES_TO_STORE*MAC_ADDRESSES_TO_STORE)+(MAC_ADDRESSES_TO_STORE*6)+30];

alignas(4) uint8_t messageData[MAC_ADDRESSES_TO_STORE /* store by mac address */][MESSAGES_TO_STORE][DATA_SIZE] = {0};
alignas(4) uint8_t messageDataBookmark[MAC_ADDRESSES_TO_STORE] = {0};
alignas(4) uint8_t messageDataSubBookmark[MAC_ADDRESSES_TO_STORE] = {0};

alignas(4) uint32_t messageTags[MESSAGE_TAGS_TO_STORE] = {0};
alignas(4) uint8_t messageTagBookmark = 0;
alignas(4) uint8_t batchTags[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS] = {0};
alignas(4) uint8_t batchTagsBookmark[MESSAGE_TAGS_TO_STORE] = {0};
// alignas(4) uint8_t batchData[MESSAGE_TAGS_TO_STORE][NO_OF_BATCH_TAGS][USABLE_TRANSMISSION_SPACE] = {0};
alignas(4) uint8_t batchDataBookmark = 0;
alignas(4) uint8_t currentPersonalBatchTag = 0;
alignas(4) MacAddress myMacAddress;

unsigned long lastDataSend = 0;

//Defining Message Types
typedef struct {
    alignas(4) MacAddress macAddress;
    uint32_t messageTag;
    uint8_t batchTag;
    alignas(4) uint8_t data[USABLE_TRANSMISSION_SPACE];
} opTransmission;
const size_t opTransmissionSize = sizeof(opTransmission);

typedef struct {
    uint32_t code;
    alignas(4) uint8_t routerAddress[6];
} scanTransmission;
const size_t scanTransmissionSize = sizeof(scanTransmission);

unsigned long lastScanBroadcastTime = 0;

// alignas(4) uint8_t data[DATA_SIZE] = {0}; //declared for send-random-data function to overcome stack allocation issues


// Structures and Variables

bool scanMode = false;
unsigned long scanEndTime = 0;

// Function Prototypes
void printMacAddress(const uint8_t* mac_addr);
void onDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len);
void initESPNow();
void handleSerialInput();
void onRequest(AsyncWebServerRequest* request);
void initTCPServer();
void handleRestart();
void handleScan();
bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum);


bool addPeer(const uint8_t* mac_addr);

// TCP server on port 80
AsyncWebServer server(80);

void setup() {
    Serial.begin(115200);
    
    // Set WiFi mode to both AP and STA
    WiFi.mode(WIFI_AP_STA);
    
    if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
        Serial.println("AP Config Failed");
    }

    WiFi.softAP(ssid, password);
    
    initESPNow();
    initTCPServer();

    Serial.println("ESP32 AP is running");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("STA MAC Address: ");
    uint8_t myAddress[6];
    WiFi.macAddress(myAddress);
    Serial.print("Mac Address: ");
    for (int i = 0;  i < 6; i++) {
        Serial.print(myAddress[i], HEX);
        if (i < 5) {
            Serial.print(":");
        }
    }
    Serial.println();

    // Add the broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
    } else {
        Serial.println("Added broadcast peer");
    }
}

void loop() {
    handleSerialInput();
    // Nothing to do here, all logic handled in callbacks and TCP server
}

void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
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

bool addPeer(const uint8_t* mac_addr) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac_addr, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;

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
    for (int i = 0; i<MAC_ADDRESSES_TO_STORE; i++) {
        if (memcmp((void*)&macAddresses[i], mac_addr, 6) == 0) {
            // Serial.println("(COMP) Incoming Address: ");
            // printMacAddress(mac_addr);
            // Serial.println("(COMP) Compared Address: ");
            // printMacAddress((uint8_t*)&macAddresses[i]);
            return true;
        }
    }
    return false;
}



void onDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
    Serial.print("Data received from MAC: ");
    printMacAddress(mac_addr);
    Serial.print("Length of message: ");
    Serial.println(len);

    uint64_t setupMessage;
    memcpy(&setupMessage, incomingData, sizeof(uint64_t));

    if (len == sizeof(uint64_t) /*&& setupMessage == SETUP_COMMAND*/) { //Check for setup message
        Serial.println("Received scan response message");
        if(macAddressBookmark<=MAC_ADDRESSES_TO_STORE){
            if(!isDuplicateAddress(mac_addr)){
                Serial.println("Isn't a duplicate address");
                if(addPeer(mac_addr)){
                    Serial.println("Added Address");
                    memcpy(&macAddresses[macAddressBookmark], mac_addr, sizeof(macAddresses[0]));
                    macAddressBookmark++;
                }
                else {
                    Serial.println("Failed to add address");
                }
            }
            else{Serial.println("Is a duplicate address");}
        }
        else{Serial.println("Out of storage, cannot add new peer");}
    }

    if(len > 5 && len < 251 && !scanMode){
        MacAddress receivedAddress;
        memcpy(&receivedAddress.address, incomingData, 6);
        uint32_t receivedTag;
        memcpy(&receivedTag, incomingData + 6, 4);
        uint8_t receivedBatchTag;
        memcpy(&receivedBatchTag, incomingData + 10, 1);
        Serial.printf("Mesh transmission received, tag: %u\n", receivedTag);

        if (!isDuplicateMessage(receivedTag, receivedBatchTag)) {
            messageTags[messageTagBookmark] = receivedTag;
            messageTagBookmark = (messageTagBookmark + 1) % MESSAGE_TAGS_TO_STORE;
            size_t receivedDataLength = len-11;
            uint8_t receivedData[receivedDataLength] = {0};
            memcpy(receivedData, incomingData+11, (receivedDataLength));
             if(!isDuplicateAddress(mac_addr)){
                if(macAddressBookmark<MAC_ADDRESSES_TO_STORE){
                memcpy((void*)&macAddresses[macAddressBookmark], (void*)&receivedAddress, 6);
                memcpy((void*)&messageData[macAddressBookmark][messageDataBookmark[macAddressBookmark]], receivedData, receivedDataLength);
                messageDataSubBookmark[macAddressBookmark] = (messageDataSubBookmark[macAddressBookmark] + receivedDataLength) % (MESSAGES_TO_STORE-1);
                macAddressBookmark++;
                }
                Serial.println("Blocked non-peer message");
            }
            else{
                Serial.println("Got a real message");
                for (int i = 0; i<macAddressBookmark-1; i++) {
                    if (memcmp((const void*)&macAddresses[i], (const void*)&receivedAddress, 6) == 0) {
                        memcpy((void*)&messageData[i][messageDataBookmark[i]], receivedData + messageDataSubBookmark[i], receivedDataLength);
                        uint16_t previousValue = messageDataSubBookmark[i];
                        messageDataSubBookmark[i] = (messageDataSubBookmark[i] + receivedDataLength) % (MESSAGES_TO_STORE-1);
                        if(messageDataSubBookmark[i] < previousValue){
                        messageDataBookmark[i] = (messageDataBookmark[i] + 1) % MESSAGES_TO_STORE;}
                    }
                    else{Serial.println("Error locating mac address (in data receipt area)");}
                }
            }
        }

    }
}

void handleRestart() {
    macAddressBookmark = 0;
    memset((void*)macAddresses, 0, sizeof(macAddresses));
    memset((void*)messageData, 0, sizeof(messageData));
    memset((void*)messageDataBookmark, 0, sizeof(messageDataBookmark));
    memset((void*)messageDataSubBookmark, 0, sizeof(messageDataSubBookmark));
    memset((void*)messageTags, 0, sizeof(messageTags));
    memset((void*)batchTags, 0, sizeof(batchTags));
    memset((void*)batchTagsBookmark, 0, sizeof(batchTagsBookmark));
    // memset((void*)batchData, 0, sizeof(batchData));
    scanMode = false;
    Serial.println("System restarted and memory cleared.");
    handleScan();
}

void handleScan() {
    Serial.println("Scanning for trusted peers...");
    scanMode = true;
    scanEndTime = millis() + SCAN_DURATION;

    uint8_t myAddress[6];
    WiFi.macAddress(myAddress);

    uint64_t setupTime = SETUP_COMMAND;
    uint8_t setupMessage[sizeof(SETUP_COMMAND)];
    memcpy(setupMessage, &setupTime, sizeof(SETUP_COMMAND));

    unsigned long lastSent = 0;
    while (millis() < scanEndTime) {
        if (millis() - lastSent > 1000) {
            esp_err_t result = esp_now_send(broadcastAddress, setupMessage, sizeof(setupMessage));
            if (result == ESP_OK) {
                Serial.println("Scan broadcast message sent successfully");
            } else {
                Serial.print("Error sending scan broadcast message: ");
                Serial.println(esp_err_to_name(result));
            }
            lastSent = millis();
        }
    }

    scanMode = false;
    Serial.println("Scan complete.");
}

void onRequest(AsyncWebServerRequest* request) {
    Serial.println("Received request");
    if (request->hasParam("message")) {
        String message = request->getParam("message")->value();
        if (message == "restart") {
            handleRestart();
        } 
        else if (message == "scan") {
            handleScan();
        }
        // size_t messageDataSize = DATA_SIZE*MESSAGES_TO_STORE*MAC_ADDRESSES_TO_STORE;
        // size_t macAddressesSize = 
        // request->send(new CustomResponse(messageData, messageDataSize, macAddresses, macAddressesSize));;
        // return;
    }


    memcpy(responseBuffer, messageData, sizeof(messageData));
    memcpy(responseBuffer+sizeof(messageData), macAddresses, sizeof(macAddresses));

    AsyncWebServerResponse *response = request->beginResponse_P(200, "application/octet-stream", responseBuffer, totalSize);
    response->addHeader("Content-Disposition", "attachment; filename=data.bin");
    request->send(response);
}

void initTCPServer() {
    server.on("/", HTTP_GET, onRequest);
    server.begin();
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "restart") {
            handleRestart();
        } else if (input == "scan") {
            handleScan();
            Serial.println("Scanning for trusted peers...");
        } else {
            Serial.println("Unknown command.");
        }
    }
}
