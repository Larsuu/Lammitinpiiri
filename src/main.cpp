/*

Ensinmäinen proseduraalinen testikoodi laitteistoa ohjaamaan ja toimintoja testaamaan. 

Seuraavaksi tehdään akusta olio.

*/

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <QuickPID.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid     = "OpenWrt";
const char* password = "10209997";
const char* mqtt_server = "192.168.1.1";

IPAddress staticIP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

#define AKKUJANNITE "akku/jannite"
#define AKKULAMPO_NTC "akku/lampoNTC"
#define AKKULAMPO_LM35 "akku/lampoLM35"
#define AKKULAMPOTEHOT "akku/tehot"
#define LAMPOKATKAISU "akku/lamposw"
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60 /* Time ESP32 will go to sleep (in seconds) */

#define HEATER 13
#define CHARGER 12
#define WDT_TIMEOUT 60          // Watchdog timeout 60 sekunttia
#define SDAPIN 4
#define SCLPIN 5

//quickpid muuttujat
bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter
float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
float Setpoint, Input, Output;
float Kp = 2, Ki = 5, Kd = 1;

// kirjastot
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_ADS1115 ads;
QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::reverse);




// ADC muuttujat
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;
int16_t mapval;
int16_t mapvalinv; 


//check? -delete
unsigned long lastmillis = 0;
unsigned long prevmillis = 0;
unsigned long lampoMillis = 0;
unsigned long interval = 30000;

// ohjelman määrityksiä nolliin. Alustusta.
float ntc1tmp = 0;
float NewSetpoint = 0;
int LampoKatkaisu = 1;

//MQTT tallennuksia aikavertailuja.
long lastMsg = 0;
char ntc_msg[20];
char lm35_msg[20];
char v_msg[20];
char bttpwr_msg[20];
char heatsw_msg[20];


void receivedCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    }
  Serial.println(" ");
}

// MQTT yhdistys funktio.
void mqttconnect() {
  /* Loop until reconnected */
  delay(500);
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic with default QoS 0*/
      client.subscribe(AKKUJANNITE);
      client.subscribe(AKKULAMPO_NTC);
      client.subscribe(AKKULAMPO_LM35);
      client.subscribe(AKKULAMPOTEHOT);
      client.subscribe(LAMPOKATKAISU);
      } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(500);
    }
  }
}

float ADC_luku()
{
  if (lastmillis - prevmillis >= 10000)
    {
      prevmillis = lastmillis;

      adc0 = ads.readADC_SingleEnded(0);  // NTC vastus 10/10 jännitejako. 20kOhm = 25C, 38kOhm=0C B=3977,3455 ehkä.  
      adc1 = ads.readADC_SingleEnded(1);  // NTV vastus. (1. sensori => 0C=32 kOhm/28 kOhm, 100C=0.9kOhm) B=36390
      adc2 = ads.readADC_SingleEnded(2);  // LM35 :n lämpömittaus
      adc3 = ads.readADC_SingleEnded(3);  // Jännitteenmittaus 3.3k/100k = Gain1(4.096V), max=127.5V, resolution=0.125mV. 

      volts0 = ads.computeVolts(adc0);
      volts1 = ads.computeVolts(adc1);
      volts2 = ads.computeVolts(adc2) * 100;    // lm35= 10mV/C 
      volts3 = ads.computeVolts(adc3) * 31.31;  // jännitteenjaon korjauskerroin

      Serial.println("-----------------------------------------------------------");
      Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
      Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
      Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println(" 'C");
      Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");
      Serial.print("Setpoint: "); Serial.print(Setpoint); Serial.print("  "); Serial.println(" 'C");
      //Serial.print("Output0: "); Serial.print(tehot); Serial.print("  "); Serial.println(" ms");



    }


return volts3;
}

// akun lukufunktio ADC:ltä.
float battery_read()
{

    uint32_t voltage = ADC_luku();

    return voltage;
   
    }

void setup()
{
  Serial.begin(115200);    
  while (!Serial) { ; }

  pinMode(HEATER, OUTPUT);                  // Lämmitinvastuksen ohjauspinni
  pinMode(CHARGER, OUTPUT);                  // Laturin rele AC
  Wire.begin(SDAPIN,SCLPIN);                    // i2c pinnit ADS1115:lle


  // QuickPID
  Input = volts1;
  Setpoint = 2; 
  myPID.SetTunings(Kp, Ki, Kd); //apply PID gains
  myPID.SetMode(myPID.Control::automatic);   //turn the PID on

  //ADS1115
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  if (!ads.begin()) {                                   // ADS:n odotus = !True
    Serial.println("Failed to initialize ADS.");
    while (1);
    }
    
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) 
      {
      Serial.println("Configuration failed.");
      }

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Subnet Mask: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("DNS 1: ");
    Serial.println(WiFi.dnsIP(0));
    Serial.print("DNS 2: ");
    Serial.println(WiFi.dnsIP(1));

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    /* configure the MQTT server with IPaddress and port */
    client.setServer(mqtt_server, 1883);
    /* this receivedCallback function will be invoked 
    when client received subscribed topic */
    client.setCallback(receivedCallback);
}

void loop()
{
  lastmillis = millis();

 volts1 = Input = ads.computeVolts(ads.readADC_SingleEnded(2));
  // Serial.print("Input: ");
  // Serial.println(Input);
  //volts1 = Input; 

  myPID.Compute();    // PID laskenta.

  /*
  Serial.print("Input: ");
  Serial.println(Input);
  
  Serial.print("Output: ");
  Serial.println(Output);
  
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  */

  analogWrite(HEATER, Output);                   // vastaa analogwrite:ä, kanava ja taajuus 8 bittisenä. 

   // Jos käydään lämpörajoilla, katkaistaan laturin virta. 
  if( volts1 < 1.5 || volts1 >  3.5 )  // 3.533=5C, 1.5=47C
    { 
      digitalWrite(CHARGER, HIGH);
      LampoKatkaisu = 1;
      //Serial.println("Lampokatkaisu=1, charger LOW");
    }
  
  if (LampoKatkaisu == 1 && volts1 < 3.38 && volts1 > 1.78 && lastmillis - lampoMillis >= 5000)  // 3.38=8C, 1.78=40C
    {
        lampoMillis = lastmillis; 
        LampoKatkaisu = 0;
        Serial.println("Lampokatkaisu=0");
    }
  if(LampoKatkaisu == 0)
  {
    digitalWrite(CHARGER, LOW);
    //serial.println("Charger HIGH");
  } 




  //ADC_luku();

  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
      mqttconnect();
    }
  
  client.loop();

  float janniteviesti;
  float NTC_viesti;  
  float LM35_viesti;
  float bttpwr_viesti;
  float heatsw_viesti;

  
  long now = millis();
  if (now - lastMsg > 10000) 
  {
    lastMsg = now;
    janniteviesti = ads.computeVolts(ads.readADC_SingleEnded(3)) * 31.31;
    NTC_viesti = ads.computeVolts(ads.readADC_SingleEnded(2));
    LM35_viesti = ads.computeVolts(ads.readADC_SingleEnded(0)) * 100;
    bttpwr_viesti = ( Output / 255 ) * 100;
    heatsw_viesti = LampoKatkaisu;
    Serial.print("LAmpokatkaisu-mainloop: ");
    Serial.println(LampoKatkaisu);

    Serial.print("Output-mainloop: ");
    Serial.println(Output);
        
    if (!isnan(janniteviesti) && !isnan(NTC_viesti) && !isnan(LM35_viesti))

    {

      Serial.print("HeatSW: ");
      Serial.println(heatsw_viesti);

      Serial.print("Tehot: ");
      Serial.println(bttpwr_viesti);
      

      snprintf (v_msg, 8, "%f", janniteviesti);
      snprintf (ntc_msg, 8, "%f", NTC_viesti);
      snprintf (lm35_msg, 8, "%f", LM35_viesti);
      snprintf (bttpwr_msg, 3, "%f", bttpwr_viesti);
      snprintf (heatsw_msg, 2, "%f", heatsw_viesti);

      Serial.print("HeatSW: ");
      Serial.println(heatsw_msg);

      Serial.print("Tehot: ");
      Serial.println(bttpwr_msg);
      


      /* publish the message */
      client.publish(AKKUJANNITE, v_msg);
      client.publish(AKKULAMPO_NTC, ntc_msg);
      client.publish(AKKULAMPO_LM35, lm35_msg);
      client.publish(AKKULAMPOTEHOT, bttpwr_msg);
      client.publish(LAMPOKATKAISU, heatsw_msg);
      Serial.println(" ");
      //delay(100);
      }
  }

//uusvanhakoodi

  
  

}