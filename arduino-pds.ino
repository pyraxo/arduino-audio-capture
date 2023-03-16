#include <Arduino.h>
#include <WiFiNINA.h>
#include <Firebase_Arduino_WiFiNINA.h>
#include <SD.h>
#include "arduino_secrets.h"
#include "ADCSetup.h"

char ssid[] = WIFI_SSID; // WiFi SSID
char pass[] = WIFI_PASS; // WiFi password
int status = WL_IDLE_STATUS;

#define WIFI_RETRY_DELAY 2000 // WiFi reconnect delay

// Initialize Firebase
FirebaseData firebaseData;

// WAV file header
PROGMEM const byte header [44] = {
  0x52, 0x49, 0x46, 0x46, 0x00, 0x00, 0x00, 0x00, 0x57, 0x41, 0x56,
  0x45, 0x66, 0x6D, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x01, 0x00, 0x40, 0x1F, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x01,
  0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00
};

// File positions for WAV file write
const byte firstPos = 4;
const byte secondPos = 40;
const byte thirdPos = 24;
const byte fourthPos = 28;

int sample_rate;
int bitsPerSample = 16;
// With a sample rate of ~16kHz, this equals to around 1 second of recording.
// Increase for longer recording.
long int num_samples = 16000; 

String fp = "/pds";
String wavFile;

#define SD_PIN 4
#define BUTTON_PIN 3
#define AUDIO_PIN A5
#define led 5

bool isRecording = false;
bool toUpload = false;
File file;
File dir;

void setup() {
  // Uncomment for fast ADC analogRead
  // ADCsetup(128, 0);
  Serial.begin(9600);
  while (!Serial) {}

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(led, LOW);

  // Initialize SD card
  while (!initSD()) {
    errorStatus();
    delay(1000);
  }

  initFirebase();
  stepStatus();
}

bool initSD() {
  // Start the MicroSD card
  Serial.println("Mounting MicroSD Card");
  pinMode(SD_PIN, OUTPUT);

  if (SD.begin(SD_PIN)) {
    Serial.println("SD init");
    File newFile = SD.open("rec000000.wav", FILE_WRITE);
    if (SD.exists("REC001.wav")) {
      if (!SD.remove("REC001.wav")) {
        Serial.println("SD card may be write protected or damaged!");
        errorStatus();
        // Any previous (or incomplete) recording should be deleted
        // in order to make a new one. This error might occur due to
        // write-protected or damaged card.
        return false;
      }
    }
    return true;
  }
  errorStatus();
  Serial.println("SD init failed!");
  return false;
}

void reconnectWiFi() {
  Serial.println("Attempting to reconnect to WiFi...");
  WiFi.disconnect();
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(WIFI_RETRY_DELAY);
    Serial.println("Attempting to reconnect to WiFi...");
  }
  if (status == WL_CONNECTED) {
    stepStatus();
    Serial.println("Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    errorStatus();
    Serial.println("Failed to connect to WiFi!");
  }
}

void initFirebase() {
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASS);
  Firebase.reconnectWiFi(true);
}

unsigned long time_now = 0;

void loop() {
  if (status != WL_CONNECTED) {
    reconnectWiFi();
  }
  if (digitalRead(BUTTON_PIN) == LOW && !isRecording) {
    Serial.println("Button pressed");
    delay(1000);
    dir = SD.open("/");
    while (true) {
      delay(100);
      File entry = dir.openNextFile();
      Serial.println(String(entry.name()));
      if (!entry) {
        entry.close();
        Serial.println("Creating new file");
        break;
      }
      // Create next numbered file i.e rec00001.wav, rec00002.wav etc.
      wavFile = String(entry.name());
      if (!wavFile.startsWith("REC")) {
        continue;
      }
      entry.close();
    }
    // Increment the recording number
    if (wavFile[7] == '9') {
      wavFile[6] = int(wavFile[6]) + 1;
		  wavFile[7] = '0';
    } else {
      wavFile[7] = int(wavFile[7]) + 1;
    }
    Serial.println(wavFile);
    file = SD.open(wavFile, FILE_WRITE);
    if (!file) {
      Serial.println("Error opening file " + wavFile);
      return;
    }
    stepStatus();
    startRecording();
  }
  // Async delay
  if (millis() >= time_now + 2000) {
    time_now += 5000;
    if (Firebase.getString(firebaseData, fp + "/latest")) {
      if (firebaseData.dataType() != "string") {
        return;
      }
      long ts = firebaseData.stringData().toInt();
      // If current time is within 6 seconds of the latest entry update,
      // light up the LED.
      if (WiFi.getTime() <= ts + 6) {
        digitalWrite(led, HIGH);
        return;
      }
      digitalWrite(led, LOW);
      return;
    }   
  }
}

void updateEntry() {
  String time = String(WiFi.getTime());
  wavFile.toLowerCase();
  wavFile.replace(".wav", "");
  // Set the latest update time to current time
  if (Firebase.setString(firebaseData, fp + "/" + wavFile, time)) {
    Serial.println("Added new entry for " + wavFile + ": " + time);
  } else {
    Serial.println("Error recording entry at " + time);
    Serial.println(firebaseData.errorReason());
  }
  if (Firebase.setString(firebaseData, fp + "/latest", time)) {
    Serial.println("Flagged latest update!");
  } else {
    Serial.println("Error updating latest timestamp");
    Serial.println(firebaseData.errorReason());
  }
}

void startRecording() {
  // Write the WAV file header
  writeHeader();

  isRecording = true;
  record();

  // Update the WAV file header with the final file size
  finalise();

  isRecording = false;

  updateEntry();
}

void writeHeader() {
  // Puts the "identification" of a typical WAV file (because .wav isn't enough)
  for (byte pos = 0; pos < sizeof(header); pos++) {
    file.write(pgm_read_byte(&header[pos]));
  }
}

void record() {
 /*
  * This is how the recording process works. This will last until (normally)
  * the press of the button.
  *
  * DO NOT RESET OR POWER-down THE BOARD WHILE IN THIS STATE, UNTIL THE STATUS LED
  * GETS IN THE FINISHED STATE; otherwise file corruption and/or SD card damage will occur.
  *
  * Exceeding 4 GiB file size (149hr 7min 50sec of audio recording), when meeting a file system's
  * limitation, or filling up the memory; may cause unexpected behavior.
  */
  int sample = 0;
  long int start = millis();
  long int time_taken;
  // Change num_samples to change record time. Current sampling rate is ~16,000 samples per sec
  long int count = num_samples;
 
  // Switch ON the LED to indicate Recording has started 
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Recording started!");
  
  while (count >= 0) {
    sample = analogRead(AUDIO_PIN);
    file.write(sample);
    count--;
  }
  // Calculate the average sampling frequency obtained
  time_taken = millis() - start;
  sample_rate = (int)((num_samples * 1000) / time_taken);
 
  Serial.println("Recording finished!");
  Serial.print("Time(s): ");
  Serial.println((float) time_taken / 1000);
  Serial.print("Sampling Frequency: ");
  Serial.println(sample_rate);

  digitalWrite(LED_BUILTIN, LOW);
}

void finalise() {
  // Fills up four necessary variables in the file's header,
  // then ensures a successfully saved recording.
  byte finalValue[4];
  unsigned long fileSize = file.size();
  unsigned long riffSize = fileSize - 8;
  unsigned long dataSize = fileSize - 44;
  int val = sample_rate * bitsPerSample / 8;

  finalValue[0] = riffSize & 0xff;
  finalValue[1] = (riffSize >> 8) & 0xff;
  finalValue[2] = (riffSize >> 16) & 0xff;
  finalValue[3] = (riffSize >> 24) & 0xff;

  file.seek(firstPos);
  file.write(finalValue, 4);
  // Check if already in little-endian order

  finalValue[0] = sample_rate & 0xff;
  finalValue[1] = (sample_rate >> 8) & 0xff;
  finalValue[2] = (sample_rate >> 16) & 0xff;
  finalValue[3] = (sample_rate >> 24) & 0xff;

  file.seek(thirdPos);
  file.write(finalValue, 4);

  finalValue[0] = val & 0xff;
  finalValue[1] = (val >> 8) & 0xff;
  finalValue[2] = (val >> 16) & 0xff;
  finalValue[3] = (val >> 24) & 0xff;
  // Is possible to make a fuction that returns an array of bytes?

  file.seek(fourthPos);
  file.write(finalValue, 4);

  finalValue[0] = dataSize & 0xff;
  finalValue[1] = (dataSize >> 8) & 0xff;
  finalValue[2] = (dataSize >> 16) & 0xff;
  finalValue[3] = (dataSize >> 24) & 0xff;

  file.seek(secondPos);
  file.write(finalValue, 4);

  file.close();
}

// Various utility functions to use LED light to display status.

void stepStatus() {
  for (int i=0; i<4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void errorStatus() {
  for (int i=0; i<2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void finishStatus() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}