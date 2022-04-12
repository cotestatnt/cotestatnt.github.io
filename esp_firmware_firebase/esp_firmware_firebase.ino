

/* 1. Define the WiFi credentials */
#define WIFI_SSID "xxxxxxxxxxxxxx"
#define WIFI_PASSWORD "xxxxxxxxxxxxxxxx"

/* 2. Define the API Key */
#define API_KEY "xxxxxxxxxxxxxxxxxxxxxxxxxxx"
/* 3. Define the RTDB URL */
#define DATABASE_URL "https://esp-stream-test-xxxxxxxx-rtdb.xxxxxxx-west1.firebasedatabase.app/"
/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "xxxxxxxxxxxx.xxxxxxxxxxx@email.com"
#define USER_PASSWORD "xxxxxxxxxxxxxxxxxxx"

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #define INP_A 14  // D5
  #define INP_B 12  // D6
  #define INP_C 13  // D7
  #define OUT_A 2   // D2
  #define OUT_B 0   // D3
  #define OUT_C 4   // D4  
#elif defined(ESP32)
  #include <WiFi.h>
  #define INP_A 14  // D13
  #define INP_B 12  // D12
  #define INP_C 13  // D13
  #define OUT_A 15  // D15
  #define OUT_B 2   // D2
  #define OUT_C 4   // D4
#endif

// Even if Firebase_ESP_Client has it's own JSON parser, I prefer this library for JSON parsing
#include <ArduinoJson.h>

#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// a struct for storing GPIO properties
struct PinGPIO {
  uint8_t pin;
  const char* label;
  bool invert;
};

// Define a list of GPIO inputs
PinGPIO Inputs[] = {
  {INP_A, "Input A", true},   /* Pin number, label, pullup */
  {INP_B, "Input B", true},
  {INP_C, "Input C", true},
};

// Define a list of GPIO outputs
PinGPIO Outputs[] = {
  {OUT_A, "Output A", true},  /* Pin number, label, active low */
  {OUT_B, "Output B", false},
  {OUT_C, "Output C", false},
};


FirebaseData stream;    // Firebase data object
FirebaseData fbdo;      // Firebase data object
FirebaseAuth auth;      // Firebase authentication object
FirebaseConfig config;  // Firebase configuration object
const char* path = "/esp-stream";

// Prepare the JSON payload with updated state of GPIO list and send to Firebase RTDB
void updateGpioList() {
  String jsonStr;
  
  // Add scope in order to destroy properly DynamicJsonDocument object
  { 
    DynamicJsonDocument doc(1024);
    JsonArray gpios = doc.createNestedArray("gpios");
    
    for (PinGPIO& input : Inputs ) {    
      JsonObject gpio = gpios.createNestedObject();
      gpio["type"] = "input";
      gpio["pin"] = input.pin;
      gpio["label"] = input.label;    
      gpio["level"] = digitalRead(input.pin);   
    }
    
    for (PinGPIO& output : Outputs ) {
      JsonObject gpio = gpios.createNestedObject();
      gpio["type"] = "output";
      gpio["pin"] = output.pin;
      gpio["label"] = output.label;    
      gpio["level"] = digitalRead(output.pin);   
      gpio["invert"] = output.invert;        
    }  
    //serializeJsonPretty(doc, Serial);    
    serializeJson(doc, jsonStr); 
  } 

  if (Firebase.ready()) {    
    FirebaseJson fb_json;
    fb_json.setJsonData(jsonStr);
    if (!Firebase.RTDB.setJSONAsync(&fbdo, path, &fb_json)) {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
      Serial.println();
    }
  }
  
}

// Return true if some GPIO has changed the state
bool updateGpioState() {
  // With this word, we will check if some of defined gpios has changed
  uint16_t gpioState = 0;
  static uint16_t gpioLast;
  size_t pos = 0;
  // Check outputs
  for (PinGPIO& output : Outputs )
    gpioState = (digitalRead(output.pin) << pos++) | gpioState;
  // Check inputs
  for (PinGPIO& input : Inputs )
    gpioState = (digitalRead(input.pin) << pos++) | gpioState;

  if (gpioState != gpioLast) {
    gpioLast = gpioState;
    return true;
  }
  return false;
}


// This is the callback function called when stream from firebase was received
void streamCallback(FirebaseStream data) {
  Serial.println("\nStream Data available...");
  Serial.println("STREAM PATH: " + data.streamPath());
  Serial.println("EVENT PATH: " + data.dataPath());
  Serial.println("DATA TYPE: " + data.dataType());
  Serial.println("EVENT TYPE: " + data.eventType());

  if (data.dataType() == "json") {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, data.jsonString());    
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    
    //serializeJsonPretty(doc, Serial);    
    String command = doc["cmds"]["cmd"].as<String>();
    if (command.equals("writeOut")) {      
      uint8_t pin = doc["cmds"]["pin"];
      bool level = doc["cmds"]["level"];
      Serial.printf("\n%s %d %d\n", command.c_str(), pin, level);            
      digitalWrite(pin, level);
    }
  }
}

void streamTimeoutCallback(bool timeout) {
  if (timeout)
    Serial.println("stream timed out, resuming...\n");

  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}

void setup() {
  // Set pinMode for input and output pins
  for (PinGPIO& input : Inputs )
    pinMode(input.pin, input.invert ? INPUT_PULLUP : INPUT);

  for (PinGPIO& output : Outputs )
    pinMode(output.pin, OUTPUT);

  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.print("\nConnected with IP: ");
  Serial.println(WiFi.localIP());

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

#if defined(ESP8266)
  //Set the size of WiFi rx/tx buffers in the case where we want to work with large data.
  stream.setBSSLBufferSize(2048, 2048);
  stream.setResponseSize(1024);
#endif

  if (!Firebase.RTDB.beginStream(&stream, path)) {
    Serial.println("\nCan't begin stream connection...");
    Serial.println("REASON: " + stream.errorReason());
    Serial.println("------------------------------------\n");
  }

  // Set callback functions for stream data
  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);
  updateGpioList();
}

void loop() {
  // True on pin state change
  if ( updateGpioState()) {
    updateGpioList();
  }
}
