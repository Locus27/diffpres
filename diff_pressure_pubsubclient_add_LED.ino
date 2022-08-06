/*
  Testing this sketch to develop a differential pressure sensor network for the stack filter system in 400HV
*/

#include <Wire.h>
#include <SparkFun_MicroPressure.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <avr/dtostrf.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
QWIICMUX myMux;

IPAddress ip(10, 1, 210, 100); // CHANGE THIS ACCORDING TO IP TABLE
IPAddress dns(0, 0, 0, 0);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);

IPAddress rpi(10, 1, 210, 10);

/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);

SparkFun_MicroPressure mpr1; // Use default values with reset and EOC pins unused
SparkFun_MicroPressure mpr2;

// MQTT connection
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

char topic_pres1[] = "laser1/pres1"; // CHANGE THIS TOO
char topic_pres2[] = "laser1/pres2"; // AND THIS

float mpress1;
float mpress2;

//indicator LEDs
#define greenLED 2
#define blueLED 3
#define redLED 4
#define yellowLED 5
//filter status LEDs
#define fsLowLED 7
#define fsMedLED 8
#define fsFullLED 9
float filterStatus;


byte mac[6];

void (* resetFunc) (void) = 0;
int cycles = 0;

void setup() {
  mqttClient.setServer("10.1.210.10", 1883);
  mqttClient.setKeepAlive(15);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(fsLowLED, OUTPUT);
  pinMode(fsMedLED, OUTPUT);
  pinMode(fsFullLED, OUTPUT);
  digitalWrite(greenLED, HIGH);
  // Initalize UART, I2C bus, and connect to the micropressure sensor
  Serial.begin(115200);
  Wire.begin();

  if (myMux.begin() == false)
  {
    Serial.println("Mux not detected. Freezing...");
    for (int i = 0; i <= 5; i++) {
    digitalWrite(redLED, HIGH);
    delay(500);
    digitalWrite(redLED, LOW);
    }
    while (1)
      ;
  }
  Serial.println("Mux detected");

  myMux.setPort(1); //Connect master to port labeled '1' on the mux

  byte currentPortNumber = myMux.getPort();
  Serial.print("CurrentPort: ");
  Serial.println(currentPortNumber);

  Serial.println("Begin scanning for I2C devices");

   myMux.setPort(1);
    if(!mpr1.begin()) { 
        Serial.println("Cannot connect to mpr1");
        for (int i = 0; i <= 5; i++) {
        digitalWrite(redLED, HIGH);
        digitalWrite(yellowLED, HIGH);
        delay(500);
        digitalWrite(redLED, LOW);
        digitalWrite(yellowLED, LOW);
        }
        while(1);
    }
  
  delay(50);
  myMux.setPort(2);
  if(!mpr2.begin()) {
        Serial.println("Cannot connect to mpr2");
        for (int i = 0; i <= 5; i++) {
        digitalWrite(redLED, HIGH);
        digitalWrite(yellowLED, HIGH);
        delay(500);
        digitalWrite(redLED, LOW);
        digitalWrite(yellowLED, LOW);
        }
        while(1);
      }
  //Serial.println("Pressure Sensors Connected");
   
  

  // start wifi

  WiFi.config(ip, dns, gateway, subnet);
  WiFi.setTimeout(120 * 1000);
 
  int wifi_status=WL_IDLE_STATUS;
  while(wifi_status != WL_CONNECTED) {
    Serial.println("Connecting to wifi");
    // login
    wifi_status = WiFi.begin("USERNAME", "PASSWORD"); // change this, obviously
    for (int i = 0; i <= 5; i++) {
        digitalWrite(blueLED, HIGH);
        delay(20);
        digitalWrite(blueLED, LOW);
        }
    delay(1000);
  }
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP()); 
  //Serial.println(WiFi.gatewayIP());
  //Serial.println(WiFi.subnetMask());
  


  // MQTT connection
  //Serial.print("Connecting to the MQTT broker at:");
  //Serial.println(rpi);
  digitalWrite(yellowLED, HIGH);
  if(!mqttClient.connect("Laser1")) { // CHANGE THIS
    Serial.print("MQTT connection failed! Error code: ");
    Serial.println(mqttClient.state());
    digitalWrite(redLED, HIGH);
    while (1);
    }
    Serial.println("MQTT client conencted"); 
    digitalWrite(yellowLED, LOW);
    digitalWrite(blueLED, HIGH);


}

void reconnect() {
  // Loop until we're reconnected
  
  int wifi_status=WL_IDLE_STATUS;
  while(wifi_status != WL_CONNECTED) {
  WiFi.disconnect();
  wifi_status = WiFi.begin("USERNAME", "PASSWORD"); // change this too
    for (int i = 0; i <= 5; i++) {
        digitalWrite(blueLED, HIGH);
        delay(20);
        digitalWrite(blueLED, LOW);
        }
    delay(1000);
  }
  while (!mqttClient.connected()) {
    Serial.print("Connecting to RasPiBroker ...");
    // Attempt to connect (clientId, username, password)
    digitalWrite(yellowLED, HIGH);
    digitalWrite(blueLED, LOW);
    if(!mqttClient.connect("Laser1")) { // CHANGE THIS
    Serial.print("MQTT connection failed! Error code: ");
    Serial.println(mqttClient.state());
    digitalWrite(redLED, HIGH);
    while (1);
    }
    Serial.println("MQTT client conencted"); 
    digitalWrite(yellowLED, LOW);
    digitalWrite(blueLED, HIGH);
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
    delay(100);
  }


void loop() {
  
  mqttClient.loop();
  if ( !mqttClient.connected() ) {
    mqttClient.disconnect();
    reconnect();
  }
  
  // Pressure 1
  digitalWrite(yellowLED, HIGH);
  myMux.setPort(1);
  char mpr1Payload[8];
  Serial.print("MPR1: ");
  Serial.print(mpr1.readPressure(),4);
  Serial.println(" PSI");
  delay(50);
  mpress1 = mpr1.readPressure();
  //Serial.println(mpress1);
  dtostrf(mpr1.readPressure(), 1, 5, mpr1Payload);
  mqttClient.publish("laser1/pres1", mpr1Payload); // CHANGE THIS
  
  delay(100);
  
  // Pressure 2
  myMux.setPort(2);
  char mpr2Payload[8];
  Serial.print("MPR2: ");
  Serial.print(mpr2.readPressure(),4);
  Serial.println(" PSI");
  delay(50);
  mpress2 = mpr2.readPressure();
  //Serial.println(mpress2);
  dtostrf(mpr2.readPressure(), 1, 5, mpr2Payload);
  mqttClient.publish("laser1/pres2", mpr2Payload); // CHANGE THIS

  filterStatus = mpress1 - mpress2;
  Serial.println(filterStatus); 
  if(filterStatus <= 0.13) {
    digitalWrite(fsLowLED, HIGH); 
    digitalWrite(fsMedLED, LOW); 
    digitalWrite(fsFullLED, LOW);
    }
  if(filterStatus >= 0.131 && filterStatus <= 0.18) {
    digitalWrite(fsLowLED, LOW); 
    digitalWrite(fsMedLED, HIGH); 
    digitalWrite(fsFullLED, LOW);
    }
  if(filterStatus >= 0.181 && filterStatus < 0.2) {
    digitalWrite(fsLowLED, LOW);
    digitalWrite(fsMedLED, LOW);
    digitalWrite(fsFullLED, HIGH);
    }
  if(filterStatus > 0.21) {
   int t = 0;
   while(t < 5) {
    digitalWrite(fsLowLED, HIGH);
    digitalWrite(fsMedLED, HIGH);
    digitalWrite(fsFullLED, HIGH);
    delay(20);
    digitalWrite(fsLowLED, LOW);
    digitalWrite(fsMedLED, LOW);
    digitalWrite(fsFullLED, LOW);
    delay(5);
    t++;
   }
  }
  digitalWrite(yellowLED, LOW);
  delay(10000);
  

}
