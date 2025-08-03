// ESP32 open server on port 10000 to receive a flaot

#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "xxxxx";
const char* password = "xxxxxx";


WiFiServer server(10000);  // server port to listen on

void setup() {
  Serial.begin(115200);
  // setup Wi-Fi network with SSID and password
  Serial.printf("Connecting to %s\n", ssid);
  Serial.printf("\nattempting to connect to WiFi network SSID '%s' password '%s' \n", ssid, password);
  // attempt to connect to Wifi network:
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  Serial.println(" listening on port 10000");
}

boolean alreadyConnected = false;  // whether or not the client was connected previously

void loop() {
  static WiFiClient client;
  static int16_t seqExpected = 0;
  if (!client)
    client = server.available();  // Listen for incoming clients
  if (client) {                   // if client connected
    if (!alreadyConnected) {
      // clead out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      alreadyConnected = true;
    }
    // if data available from client read and display it
    int length;
    float value;
    if ((length = client.available()) > 0) {
      //str = client.readStringUntil('\n');  // read entire response
      Serial.printf("Received length %d - ", length);
      // if data is correct length read and display it
      if (length == sizeof(value)) {
        client.readBytes((char*)&value, sizeof(value));
        Serial.printf("value %f \n", value);
      } else
        while (client.available()) Serial.print((char)client.read());  // discard corrupt packet
    }
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("\nSSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}