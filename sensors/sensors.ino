#include <WiFi.h>

// Replace with your network credentials
<<<<<<< HEAD
const char* ssid = "TelstraOC72ML";
const char* password = "1432417514";

=======
const char* ssid = "Telstra0C72ML";
const char* password = "1432417514";
>>>>>>> 664e393500df0ef9f07320d5347b990f27f8e1b7

WiFiServer server(10000);  // server port to listen on

// Define Serial1 pins (adjust as needed)
#define SERIAL1_RX 16
#define SERIAL1_TX 17

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX); // Second serial port

  Serial.printf("Connecting to %s\n", ssid);
  Serial.printf("\nattempting to connect to WiFi network SSID '%s' password '%s' \n", ssid, password);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  server.begin();
  printWifiStatus();
  Serial.println(" listening on port 10000");
}

boolean alreadyConnected = false;

void loop() {
  static WiFiClient client;
  if (!client)
    client = server.available();
  if (client) {
    if (!alreadyConnected) {
      client.flush();
      Serial.println("We have a new client");
      alreadyConnected = true;
    }
    int length;
    float values[4];
    if ((length = client.available()) >= sizeof(values)) {
      Serial.printf("Received length %d - ", length);
<<<<<<< HEAD
      // if data is correct length read and display it
      if (length == sizeof(value)) {
        client.readBytes((char*)&value, sizeof(value));
        Serial.printf("value %f \n", value);
      } else
        while (client.available()) Serial.print(client.read());  // discard corrupt packet
=======
      size_t bytesRead = client.readBytes((char*)values, sizeof(values));
      if (bytesRead == sizeof(values)) {
        Serial.printf("values: %f, %f, %f, %f\n", values[0], values[1], values[2], values[3]);
        // Send the 4 floats as raw bytes over Serial2
        Serial1.printf("%f, %f, %f, %f\n", values[0], values[1], values[2], values[3]);
      } else {
        Serial.println("Error: Did not read 16 bytes for 4 floats.");
      }
      while (client.available()) client.read();
        } else if (length > 0) {
      while (client.available()) client.read();
>>>>>>> 664e393500df0ef9f07320d5347b990f27f8e1b7
    }
  }
}

void printWifiStatus() {
  Serial.print("\nSSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
