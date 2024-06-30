#include <WiFi.h>
#include <HTTPClient.h>
#include <LiquidCrystal.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>

const char *ssid = "IOT";
const char *password = "30010231";

const int BUFFER_SIZE = 14;       // RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const int DATA_SIZE = 10;         // 10byte data (2byte version + 8byte tag)
const int DATA_VERSION_SIZE = 2;  // 2byte version (actual meaning of these two bytes may vary)
const int DATA_TAG_SIZE = 8;      // 8byte tag
const int CHECKSUM_SIZE = 2;      // 2byte checksum

HardwareSerial ssrfid(1); // Use Serial1 on GPIO3 (RX) and GPIO1 (TX) for RFID
HardwareSerial gpsSerial(2); // Use Serial2 on GPIO16 (RX) and GPIO17 (TX) for GPS
// Define the RX and TX pins for the GPS module
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

TinyGPSPlus gps;

uint8_t buffer[BUFFER_SIZE];  // used to store an incoming data frame
int buffer_index = 0;
boolean multipleRead = false;
unsigned long lastTagTime = 0;

float latitude = 0.0;   // Declare latitude as a global variable
float longitude = 0.0;  // Declare longitude as a global variable

const int rs = 32, en = 33, d4 = 25, d5 = 26, d6 = 27, d7 = 23;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int Buzzer = 21;

WiFiClientSecure client;  // Declare the client instance globally

void setup() {
  Serial.begin(9600);
  ssrfid.begin(9600, SERIAL_8N1, 3, 1); // Using GPIO3 (RX) and GPIO1 (TX) for RFID
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Using defined GPS pins
  lcd.begin(16, 2);
  WiFi.begin(ssid, password);

  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);
  lcd.clear();
  lcd.setCursor(5,0);
  lcd.print("SYSTEM");
  lcd.setCursor(1,1);
  lcd.print("INITIALIZATION");
  delay(3000);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("WiFi: Connecting");
    lcd.setCursor(0,1);
    lcd.print("IP: Loading...");
    delay(3000);
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("WiFi: Connected");
  lcd.setCursor(0,1);
  lcd.print("IP: " + WiFi.localIP().toString());
  delay(3000);

  // Serial.println("WiFi connected");
  // Serial.println("IP address: " + WiFi.localIP().toString());
}
void loop() {
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("SCAN TAG");
  delay(1000);
  digitalWrite(Buzzer, LOW);
  // lcd.setCursor(0,1);
  // lcd.print("");
  long tagID = 0;
    // Read GPS data continuously
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  // Read GPS data continuously
  while (ssrfid.available() > 0) {
    bool call_extract_tag = false;

    int ssvalue = ssrfid.read();  // read
    if (ssvalue == -1) {          // no data was read
      return;
    }

    if (ssvalue == 2) {  // RDM630/RDM6300 found a tag => tag incoming
      buffer_index = 0;
    } else if (ssvalue == 3) {  // tag has been fully transmitted
      call_extract_tag = true;  // extract tag at the end of the function call
      multipleRead = true;
    }

    if (buffer_index >= BUFFER_SIZE) {  // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
      Serial.println("Error: Buffer overflow detected!");
      return;
    }

    buffer[buffer_index++] = ssvalue;  // everything is alright => copy current value to buffer

    if (call_extract_tag == true) {
      if (buffer_index == BUFFER_SIZE) {
        unsigned tag = extract_tag();
      } else {  // something is wrong... start again looking for preamble (value: 2)
        buffer_index = 0;
        return;
      }
    }
  }

  // Check if a valid GPS fix is available
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    
    // Print GPS data for debugging
    // Serial.print("Latitude: ");
    // Serial.println(latitude, 6);
    // Serial.print("Longitude: ");
    // Serial.println(longitude, 6);
    // delay(1000);

    // Call the sendTagToServer function here to send the tag and GPS data
    sendTagToServer(tagID, latitude, longitude);
  }

  // Check if a timeout occurred and reset the buffer if needed
  if (millis() - lastTagTime > 5000) {  // Adjust the timeout value as needed (5000 milliseconds = 5 seconds)
    buffer_index = 0;
    lastTagTime = millis();
    multipleRead = false;
  }

  if (multipleRead) {
    while (ssrfid.available() > 0) {
      int ssvalue = ssrfid.read();  // read
        // Get GPS data
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        // Serial.println(latitude, 6);
        // delay(1000);
        // Serial.println(longitude, 6);
        // delay(1000);
            // Send tag and GPS data to server
        sendTagToServer(tagID, latitude, longitude);
      }
      if (ssvalue == -1) {          // no data was read
        break;
      }
    }
    for (int x = 0; x < 14; x++) {
      buffer[x] = 0;
    }
    multipleRead = false;
  }
}

unsigned extract_tag() {
  uint8_t msg_head = buffer[0];
  uint8_t *msg_data = buffer + 1;  // 10 byte => data contains 2byte version + 8byte tag
  uint8_t *msg_data_version = msg_data;
  uint8_t *msg_data_tag = msg_data + 2;
  uint8_t *msg_checksum = buffer + 11;  // 2 byte
  uint8_t msg_tail = buffer[13];

  // print message that was sent from RDM630/RDM6300
  Serial.println("--------");

  Serial.print("Message-Head: ");
  Serial.println(msg_head);

  Serial.println("Message-Data (HEX): ");
  for (int i = 0; i < DATA_VERSION_SIZE; ++i) {
    Serial.print(char(msg_data_version[i]));
  }
  Serial.println(" (version)");
  for (int i = 0; i < DATA_TAG_SIZE; ++i) {
    Serial.print(char(msg_data_tag[i]));
  }
  Serial.println(" (tagID)");

  Serial.print("Message-Checksum (HEX): ");
  for (int i = 0; i < CHECKSUM_SIZE; ++i) {
    Serial.print(char(msg_checksum[i]));
  }
  Serial.println("");

  Serial.print("Message-Tail: ");
  Serial.println(msg_tail);

  Serial.println("--");

  long tagID = hexstr_to_value(msg_data_tag, DATA_TAG_SIZE);
  Serial.print("Extracted Tag: ");
  Serial.println(tagID);

  // Send the tag ID to the server
  sendTagToServer(tagID, latitude, longitude);
  delay(1000);

  long checksum = 0;
  for (int i = 0; i < DATA_SIZE; i += CHECKSUM_SIZE) {
    long val = hexstr_to_value(msg_data + i, CHECKSUM_SIZE);
    checksum ^= val;
  }
  Serial.print("Extracted Checksum (HEX): ");
  Serial.print(checksum, HEX);
  if (checksum == hexstr_to_value(msg_checksum, CHECKSUM_SIZE)) {  // compare calculated checksum to retrieved checksum
    Serial.print(" (OK)");                                         // calculated checksum corresponds to transmitted checksum!
  } else {
    Serial.print(" (NOT OK)");  // checksums do not match
  }

  Serial.println("");
  Serial.println("--------");

  return tagID;
}
long hexstr_to_value(uint8_t *str, unsigned int length) {  // converts a hexadecimal value (encoded as ASCII string) to a numeric value
  uint8_t *copy = (uint8_t *)malloc((sizeof(uint8_t) * length) + 1);
  memcpy(copy, str, sizeof(uint8_t) * length);
  copy[length] = '\0';
  // the variable "copy" is a copy of the parameter "str". "copy" has an additional '\0' element to make sure that "str" is null-terminated.
  long value = strtol((char *)copy, NULL, 16);  // strtol converts a null-terminated string to a long value
  free(copy);                                   // clean up
  return value;
}
void sendTagToServer(long tagID, float latitude, float longitude) {
  static unsigned long lastSendTime = 0;   // Keep track of the last time data was sent
  static long lastTagID = -1;              // Keep track of the last sent tagID
  static unsigned long lastTagScanTime = 0; // Keep track of the time when the last tag was scanned

  // Check if a tag is scanned
  if (tagID != 0) {
    // Check if the same tag is scanned within the 30-second interval
    if (tagID == lastTagID && millis() - lastTagScanTime < 30000) {
      Serial.println("Same tag scanned within 30 seconds, data not sent.");
      return; // Do not send the data
    }
    // Check if latitude and longitude are valid and non-zero
    if(latitude != 0.0 && longitude != 0.0) {
      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient http;
      String serverURL = "https://safa.co.ke/upload.php";
      http.begin(client, serverURL);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // Prepare data to send to the server
      String postData = "tag=" + String(tagID) + "&latitude=" + String(latitude, 6) + "&longitude=" + String(longitude, 6);

      int httpResponseCode = http.POST(postData);
      if (httpResponseCode > 0) {
        digitalWrite(Buzzer, LOW);
        Serial.printf("Tag sent to server. HTTP Response code: %d\n", httpResponseCode);
        Serial.println("Tag Send Successfully");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("TAG: ");
        lcd.setCursor(5,0);
        lcd.print(tagID);
        lcd.setCursor(0,1);
        lcd.print("SENDING...");
        delay(500);
        lastSendTime = millis();       // Update the last send time
        lastTagID = tagID;             // Update the last sent tagID
        lastTagScanTime = lastSendTime; // Update the last tag scan time

        // Wait for the server response
        while (!client.available()) {
          delay(10);
        }

        // Parse the JSON response
        DynamicJsonDocument jsonBuffer(256);
        String response = client.readString();
        DeserializationError error = deserializeJson(jsonBuffer, response);

        if (error) {
          Serial.println("JSON parsing error");
        }

        // Check if the tagID is available in the database
        bool tagAvailable = jsonBuffer["tag_available"];

        if (tagAvailable) {
          digitalWrite(Buzzer, LOW);
          Serial.println("Tag Found!");
          lcd.clear();
          lcd.setCursor(4,0);
          lcd.print("TAG FOUND");
          lcd.setCursor(0,1);
          lcd.print("Name: ");
          lcd.print(jsonBuffer["student_lastname"].as<String>());
          delay(1000); // Display the name for 5 seconds
        } else {
          digitalWrite(Buzzer, HIGH);
          Serial.println("Tag Not Found");
          lcd.clear();
          lcd.setCursor(2,0);
          lcd.print("TAG NOT FOUND");
          delay(500);
        }
      } else {
        digitalWrite(Buzzer, HIGH);
        Serial.println("Error sending tag to server.");
        lcd.clear();
        lcd.setCursor(6,0);
        lcd.print("ERROR");
        lcd.setCursor(3,1);
        lcd.print("SENDING DATA");
        delay(500);
      }
      http.end();
    } else {
      digitalWrite(Buzzer, LOW);
      Serial.println("Invalid or zero latitude/longitude, data not sent to server.");
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print("INVALID");
      lcd.setCursor(2,1);
      lcd.print("LOCATION INFO");
      delay(500);
    }
  } else {
    Serial.println("Tag ID is null, data not sent to server.");
    delay(500);
  }
}
