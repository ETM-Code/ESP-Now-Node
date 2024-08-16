#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <cstring>
#include <FastCRC.h>

FastCRC32 CRC32;

// Definitions

#define SETUP_COMMAND 2069783108202043734ULL
#define WIFI_CHANNEL 1
#define MAC_ADDRESSES_TO_STORE 2
#define SCAN_DURATION 3000
bool initialised = false;
#define DATA_SIZE 6400
#define TRANSMISSION_SIZE 250 //Maximum bytes transmissable by ESP_Now in a single batch
#define USABLE_TRANSMISSION_SPACE 234 //Allocate 4 bytes for message identifier, 1 for batch identifier, 6 for mac address
#define TRANSMISSION_THRESHOLD 5
#define MAX_WS_CLIENTS 4
#define CLIENT_TIMEOUT 5

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




#define NO_OF_BATCH_TAGS 28
#define MESSAGES_TO_STORE 3
#define BATCH_SIZE 234

// WiFi Credentials
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char* ssid = "ESP32_Network";
const char* password = "password123";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

typedef struct {
     uint8_t address[6];
     bool sent;
} MacAddress;


 MacAddress macAddresses[MAC_ADDRESSES_TO_STORE] = {0};
alignas(4) uint8_t macAddressBookmark = 0;

size_t totalSize = (DATA_SIZE*MESSAGES_TO_STORE*MAC_ADDRESSES_TO_STORE)+(MAC_ADDRESSES_TO_STORE*8);
uint8_t responseBuffer[DATA_SIZE+sizeof(MacAddress)+sizeof(bool)/*+sizeof(uint32_t)*/+1000];

typedef struct {
    MacAddress address;
    bool notSent;
    uint8_t data[DATA_SIZE];
} dataStore;


dataStore messageData[MAC_ADDRESSES_TO_STORE /* store by mac address */][MESSAGES_TO_STORE] = {0};
 uint8_t messageDataBookmark[MAC_ADDRESSES_TO_STORE] = {0};
 int messageDataSubBookmark[MAC_ADDRESSES_TO_STORE] = {0};

uint32_t messageTags[MAC_ADDRESSES_TO_STORE][MESSAGES_TO_STORE] = {0};
 uint8_t messageTagBookmark[MAC_ADDRESSES_TO_STORE] = {0};
 uint8_t batchTagsBookmark[MAC_ADDRESSES_TO_STORE][MESSAGES_TO_STORE] = {0};
uint8_t batchTags[MAC_ADDRESSES_TO_STORE][MESSAGES_TO_STORE][NO_OF_BATCH_TAGS] = {0};
 uint8_t currentPersonalBatchTag = 0;
 MacAddress myMacAddress;

 std::vector<std::pair<AsyncWebSocketClient *, unsigned long>> clients;

unsigned long lastDataSend = 0;

//Defining Message Types
typedef struct {
     MacAddress macAddress;
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

//  uint8_t data[DATA_SIZE] = {0}; //declared for send-random-data function to overcome stack allocation issues


// Structures and Variables

bool scanMode = false;
unsigned long scanEndTime = 0;

// Function Prototypes
void printMacAddress(const uint8_t* mac_addr);
void onDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len);
void initESPNow();
void handleSerialInput();
void onRequest(AsyncWebServerRequest* request);
// void initTCPServer();
void initWebSocket();
void handleRestart();
void handleScan();
bool isDuplicateBatch(uint32_t messageTag, uint8_t batchTag, int messageTagNum);


bool addPeer(const uint8_t* mac_addr);

// TCP server on port 80
// AsyncWebServer server(80);

// Web Socket Server on Port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setup() {
    Serial.begin(115200);
    
    // Set WiFi mode to both AP and STA
    WiFi.mode(WIFI_AP_STA);
    
    if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
        Serial.println("AP Config Failed");
    }

    WiFi.softAP(ssid, password);
    WiFi.setSleep(false);
    
    initESPNow();
    // initTCPServer();
    initWebSocket();
    server.begin();

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
    delay(3000);
    handleRestart();
}

void loop() {
    handleSerialInput();
    // Serial.print("Free Memory: ");
    // Serial.println(esp_get_free_heap_size());
//     if (WiFi.softAPgetStationNum() == 0) {
//     WiFi.softAPdisconnect(true);
//     WiFi.softAP(ssid, password);
//     Serial.println("Restarted Wi-Fi AP");
// }
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
    esp_now_del_peer(mac_addr);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac_addr, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;

}


int findMacAddress(MacAddress receivedAddress){
    Serial.print("Mac Address Bookmark:  ");
    Serial.println(macAddressBookmark);
    for (int i = 0; i<macAddressBookmark; i++) {
        // Serial.println("Local Mac Address:  ");
        // printMacAddress((uint8_t*)&macAddresses[i]);
        // Serial.println("Received Mac Address:  ");
        // printMacAddress((uint8_t*)&receivedAddress);

        if (memcmp((const void*)&macAddresses[i], (const void*)&receivedAddress, 6) == 0) {
            return(i);
        }
    }
    return(-1);
}

int findMessageTag(uint32_t receivedTag, int addressIndex){
    for(int j = 0; j<MESSAGES_TO_STORE; j++){
        if (memcmp((const void*)&messageTags[addressIndex][j], (const void*)&receivedTag, 4) == 0){
            return(j);
        }
    }
    return(-1);
}

int findBatchTag(uint8_t receivedBatchTag, int messageIndex, int addressIndex){
    for(int k = 0; k<NO_OF_BATCH_TAGS; k++){
    if (batchTags[addressIndex][messageIndex][k] == receivedBatchTag){
            return(k);
        }
    } 
    return(-1);
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

int count = 0;
int innerCount = 0;

void onDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
    // Serial.println("Got data");
    // Serial.print("Data received from MAC: ");
    // printMacAddress(mac_addr);
    // Serial.print("Length of message: ");
    // Serial.println(len);

    uint64_t setupMessage;
    memcpy(&setupMessage, incomingData, sizeof(uint64_t));

    if (len == sizeof(uint64_t) /*&& setupMessage == SETUP_COMMAND*/&& scanMode) { //Check for setup message
        Serial.println("Received scan response message");
        if(macAddressBookmark<=MAC_ADDRESSES_TO_STORE){
            if(!isDuplicateAddress(mac_addr)){
                Serial.println("Isn't a duplicate address");
                if(addPeer(mac_addr)){
                    Serial.println("Added Address");
                    memcpy(&macAddresses[macAddressBookmark], mac_addr, 6);
                    for(int i=0; i<MESSAGES_TO_STORE; i++){
                        memcpy((void*)&messageData[macAddressBookmark][i].address, mac_addr, 6);
                    }
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

    else if(len > 50 && len < 251 && !scanMode && initialised){
        count++;
        Serial.print("Count:  ");
        Serial.println(count);
        Serial.println("Got real data");
        MacAddress receivedAddress;
        memcpy(&receivedAddress, incomingData, 6);
        uint32_t receivedTag;
        memcpy(&receivedTag, incomingData + 8, 4);
        uint8_t receivedBatchTag;
        memcpy(&receivedBatchTag, incomingData + 12, 1);
        // Serial.print("Received Batch Tag: ");
        // Serial.println(receivedBatchTag);
        // Serial.printf("Mesh transmission received, tag: %u\n", receivedTag);
        size_t receivedDataLength = len-16;
        uint8_t receivedData[receivedDataLength] = {0};

        memcpy(receivedData, incomingData+16, (receivedDataLength));

            // Serial.println("Got a real message");
            // Serial.print("macAddressBookmark:  ");
            // Serial.println(macAddressBookmark);

        if(findMacAddress(receivedAddress) != -1){
            Serial.println("Found mac address");
            int addressIndex = findMacAddress(receivedAddress);
            if(findMessageTag(receivedTag, addressIndex) != -1){
                Serial.println("Found Message Tag");
                int messageTagIndex = findMessageTag(receivedTag, addressIndex);
                if(findBatchTag(receivedBatchTag, messageTagIndex, addressIndex) != -1){
                    Serial.println("Found batch tag"); Serial.print("Batch Tag:  "); Serial.println(receivedBatchTag);
                    Serial.print("All Batch Tags:  ");
                    for(int k = 0; k<sizeof(batchTags[addressIndex][messageTagIndex]); k++){
                        Serial.print(batchTags[addressIndex][messageTagIndex][k]);
                        Serial.print(", ");
                    }
                    Serial.println();}
                else{
                    Serial.println("Couldn't find batch tag");
                    Serial.print("Batch Tag:  "); Serial.println(receivedBatchTag);
                    if(receivedBatchTag<NO_OF_BATCH_TAGS){
                        Serial.print("Inner Count:  ");
                        innerCount++;
                        Serial.println(innerCount);
                        // Serial.print("ReceivedData:  ");
                        // for(int i=0; i<receivedDataLength; i++){
                        //     Serial.print(receivedData[i]);
                        //     Serial.print(", ");
                        // }
                        Serial.println();
                        if(receivedBatchTag*BATCH_SIZE+receivedDataLength<=sizeof(messageData[addressIndex][messageTagIndex])){
                        memcpy((void*)&(messageData[addressIndex][messageTagIndex].data[receivedBatchTag*BATCH_SIZE]), receivedData, receivedDataLength);
                        batchTagsBookmark[addressIndex][messageTagIndex]++;
                        if(batchTagsBookmark[addressIndex][messageTagIndex]>=NO_OF_BATCH_TAGS-1){
                            batchTagsBookmark[addressIndex][messageTagIndex]=0;
                            messageData[addressIndex][messageTagIndex].notSent = true;
                        }
                        }
                        else{
                            Serial.println("Message didn't fit");
                            batchTagsBookmark[addressIndex][messageTagIndex]=0;
                            messageData[addressIndex][messageTagIndex].notSent = true;
                        }
                        
                    }

                }
            }
            else{
                Serial.println("Couldn't find message tag");
                Serial.print("Inner Count:  ");
                innerCount++;
                Serial.println(innerCount);
                messageTags[addressIndex][messageTagBookmark[addressIndex]] = receivedTag;
                batchTagsBookmark[addressIndex][messageTagBookmark[addressIndex]] = 0;
                memset(batchTags[addressIndex][messageTagBookmark[addressIndex]], 0, NO_OF_BATCH_TAGS);
                memset((void*)&messageData[addressIndex][messageTagBookmark[addressIndex]], 0, MESSAGES_TO_STORE);
                messageTagBookmark[addressIndex] = (messageTagBookmark[addressIndex]+1) % MESSAGES_TO_STORE;
            }
        }
        else{ //Uncomment if you want to allow any source of transmission
            Serial.println("Unrecognised Mac Address");
            // if(macAddressBookmark<MAC_ADDRESSES_TO_STORE){
            //     memcpy((void*)&macAddresses[macAddressBookmark], (void*)&receivedAddress, 6);
            //     memcpy((void*)&messageData[macAddressBookmark][messageDataBookmark[macAddressBookmark]], receivedData, receivedDataLength);
            //     messageDataSubBookmark[macAddressBookmark] = (messageDataSubBookmark[macAddressBookmark] + receivedDataLength) % (DATA_SIZE-1);
            //     macAddressBookmark++;
            // }
        }
    }
            // Serial.println();
                // Serial.println();
                // Serial.print("Arary so far:  ");
                // for(int j = 0; j<DATA_SIZE; j++){
                //     Serial.print(messageData[0][0][j]);
                //     Serial.print(" ");
                // }
                // Serial.println();
                // Serial.println();
                // Serial.print("Received Data: ");
                //     for(int j = 0; j<sizeof(receivedData); j++){
                //         Serial.print(receivedData[j]);}
                //     Serial.println();
                // }
}
                
            


void handleRestart() {
    Serial.println("Restarting");
    server.end();
    ws.closeAll();
    ws._cleanBuffers();
    initWebSocket();
    server.begin();
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
    initialised = false;
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
    initialised = true;
}

bool messagesToSend[MAC_ADDRESSES_TO_STORE][MESSAGES_TO_STORE] = {0};

bool checkForSpikes(int addressIndex, int messageIndex){
    Serial.println("Checking for spikes");
    if(addressIndex<macAddressBookmark && messageIndex<MESSAGES_TO_STORE){
        Serial.println("if'd correctly");
    for(int j=0; j<DATA_SIZE/2; j+=117){
    for(int i = 0; i<DATA_SIZE/2; i++){
        if(messageData[addressIndex][messageIndex].data[i]>=TRANSMISSION_THRESHOLD){
            return true;
        }
    }}
    }
    return false;
}


uint8_t sendAddressBuffer[10] = {0};
bool breakLoops = false;
MacAddress emptyAddress = {0};
unsigned long lastTimeTime = 0;

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

    if (msg.indexOf("restart") != -1) {
        Serial.println("Restarting from message");
        handleRestart();
    } else if (msg.indexOf("scan") != -1) {
        handleScan();
    } else if (msg.indexOf("getData") != -1) {  // Check if "getData" is contained within msg
        Serial.println("Got request for data");
        if(lastTimeTime-millis()>500){
            lastTimeTime=millis();
        for(int addressIndex=0; addressIndex<macAddressBookmark; addressIndex++){
            Serial.println("Looping through addresses");
            // if(breakLoops){breakLoops = false; break;}
            for(int messageIndex=0; messageIndex<MESSAGES_TO_STORE; messageIndex++){
                // if(breakLoops){breakLoops = false; break;}
                if(messageData[addressIndex][messageIndex].notSent==true){
                    Serial.println("We're checking to send data");
                    if(checkForSpikes(addressIndex, messageIndex)){
                        Serial.println("We're going to send data");
                        size_t dataSize = DATA_SIZE + sizeof(MacAddress) + sizeof(bool)-2;
                        memcpy(responseBuffer, (void*)&messageData[addressIndex][messageIndex], dataSize);
                        // uint32_t checksum = CRC32.crc32((uint8_t*)&messageData[addressIndex][messageIndex], dataSize);
                        // memcpy(responseBuffer + dataSize, &checksum, sizeof(checksum));
                        Serial.print("Data to send:  ");
                        bool isAccel = true;
                        for(int j=0; j<sizeof(responseBuffer)-117; j+=117){
                            if(isAccel){
                                Serial.print("Acceleration:  ");
                                isAccel = false;
                            }
                            else{Serial.print("Rotational Velocity:  "); isAccel = true;}
                            for(int i=0; i<117; i++){
                                Serial.print(responseBuffer[i+j]);
                                Serial.print(", ");
                        }
                        }
                        
                        client->binary(responseBuffer, dataSize/*+sizeof(checksum)*/);
                        memset(responseBuffer, 0, sizeof(responseBuffer));
                        Serial.print("Sent data");
                        breakLoops = true;
                        break;
                    }
                    memset((void*)&messageData[addressIndex][messageIndex], 0, sizeof(messageData));
                }
            }
        if(!macAddresses[addressIndex].sent && memcmp(macAddresses[addressIndex].address, &emptyAddress, 6) == 0){
            macAddresses[addressIndex].sent = true;
            Serial.println("Sent Mac Address");
            memcpy(sendAddressBuffer, macAddresses[addressIndex].address, 6);
            uint32_t checksum = CRC32.crc32((uint8_t*)&macAddresses[addressIndex].address, 6);
            memcpy(sendAddressBuffer+6, &checksum, 4);
            client->binary(sendAddressBuffer, sizeof(sendAddressBuffer));
            memset(sendAddressBuffer, 0, sizeof(sendAddressBuffer));
            break;
        }
    }
        }

        // Serial.print("responseBuffer: ");
        // for (int i = 0; i < 2500; i++) {
        //     Serial.print(responseBuffer[i]);  // Print each byte as a number
        //     Serial.print(" ");  // Add a space between numbers for readability
        // }
        // Serial.println();
            }
    }
}

// void initTCPServer() {
//     server.on("/", HTTP_GET, onRequest);
//     server.begin();
// }


void checkClientTimeout() {
    unsigned long currentMillis = millis();
    for (auto it = clients.begin(); it != clients.end();) {
        if (currentMillis - it->second >= CLIENT_TIMEOUT) {
            it->first->close();
            it = clients.erase(it); // Remove and disconnect the client
        } else {
            ++it;
        }
    }
}

void initWebSocket() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input == "restart") {
            Serial.println("Restarting from console");
            handleRestart();
        } else if (input == "scan") {
            handleScan();
            Serial.println("Scanning for trusted peers...");
        } else {
            Serial.println("Unknown command.");
        }
    }
}
