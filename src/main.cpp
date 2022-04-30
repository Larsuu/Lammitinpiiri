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
#define AKKU_BOOST "akku/boost_sw"
#define LAMMTOT "akku/lamm_tot"
#define UPTIMES "akku/uptimes"
#define BOOSTSENSE "akku/boostsens"
#define KP "akku/kp"
#define KI "akku/ki"

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60 /* Time ESP32 will go to sleep (in seconds) */

#define HEATER 13     //GPIO13 = D7
#define CHARGER 12    //GPIO12 = D6
#define WDT_TIMEOUT 60          // Watchdog timeout 60 sekunttia
#define SDAPIN 4
#define SCLPIN 5

//quickpid muuttujat
bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter
float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0
float Setpoint, Input, Output;
float Kp = 0.5, Ki = 0.005, Kd = 0;  // edellinen: P=0.01, I=0.05 
float lammitys_tot;

float janniteviesti;
float NTC_viesti;  
float LM35_viesti;
float bttpwr_viesti;
float heatsw_viesti;
float boost_viesti;
float lammtot_viesti;
float millis_viesti;
float bsense_viesti;
float kp_viesti;
float ki_viesti;

// kirjastot
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_ADS1115 ads;
QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);

// ADC muuttujat
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3, akunjannite;
int16_t mapval;
int16_t mapvalinv; 



//check? -delete
unsigned long lastmillis = 0;
unsigned long prevmillis = 0;
unsigned long lampoMillis = 0;
unsigned long mittausmillit = 0;
unsigned long interval = 30000;

// ohjelman määrityksiä nolliin. Alustusta.
float ntc1tmp = 0;
float NewSetpoint = 0;
int LampoKatkaisu = 1;
int akku_boost = 0;
float millisek;
int akkuboostsensor = 0;

//MQTT tallennuksia aikavertailuja.
long lastMsg = 0;
char ntc_msg[20];
char lm35_msg[20];
char v_msg[20];
char bttpwr_msg[20];
char heatsw_msg[20];
char boost_msg[20];
char lammtotmsg[20];
char ms_msg[20];
char boostsens_msg[20];
char kp_msg[20];
char ki_msg[20];




void receivedCallback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message received: ");
  Serial.println(topic);
  String topicstr = topic;
  String Lasti = String(( char *) payload); 
  Serial.println("String payload: ");
  Serial.print(topicstr);
  Serial.print("  ");
  Serial.print(Lasti);
  
  if(topicstr == AKKU_BOOST )
      {
      if(payload[0] == 49) // ASCII: 49 = 1, 48 = 0
      {
      akku_boost = 1;
      Serial.println(payload[0]);
      Serial.println("Boost on päällä");
      }
    if(payload[0] == 48)
      {
      akku_boost = 0;
      Serial.println("Boost on pois päältä");
      Serial.println(payload[0]);
      }
    // Serial.print(" akunboost_looppi");
    // Serial.println(payload[0]);
  }

  if(topic == KP)
    {
        Serial.print("Kp was: ");
        Serial.print(Kp);
        Kp = Lasti.toFloat();
        Serial.print("Kp is now: ");
        Serial.print(Kp);
    }

  if(topic == KI)
    {
        Serial.print("Ki was: ");
        Serial.print(Ki);
        Ki = Lasti.toFloat();
        Serial.print("Ki is now: ");
        Serial.print(Ki);
    }

  
  Serial.print("char payload: ");
  for (int i = 0; i < length; i++) 
    {
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
      client.subscribe(AKKU_BOOST);
      client.subscribe(LAMMTOT);
      client.subscribe(UPTIMES);
      client.subscribe(BOOSTSENSE);
      client.subscribe(KP);
      client.subscribe(KI);
      } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(500);
    }
  }
}


void setup()
{
  Serial.begin(115200);    
  while (!Serial) { ; }

  pinMode(HEATER, OUTPUT);                  // Lämmitinvastuksen ohjauspinni
  pinMode(CHARGER, OUTPUT);                  // Laturin rele AC
  Wire.begin(SDAPIN, SCLPIN);                    // i2c pinnit ADS1115:lle



  // QuickPID
  Setpoint = 24; 
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

  if(lastmillis - mittausmillit >= 1000)
    {
      mittausmillit = lastmillis;
      volts1 = ads.computeVolts(ads.readADC_SingleEnded(2));
      Input = ads.computeVolts(ads.readADC_SingleEnded(0)) * 100;
      akunjannite = ads.computeVolts(ads.readADC_SingleEnded(3)) * 31.8;
    }
 
  myPID.Compute();    // PID laskenta. Output = 8 Bit.
  
  //---------------------------------
  //  HUOMIO::HUOMIO::HUOMIO::HUOMIO
  //
  // HUOM: volts1 on invertoitu! 
  // 
  //  HUOMIO::HUOMIO::HUOMIO::HUOMIO
  //----------------------------------

  if( volts1 >= 1.2 && akunjannite >= 50)  // estetään lämpeneminen liian kuumana ja liian matalassa jännitteessä.
  {
    analogWrite(HEATER, Output);
    lammitys_tot=1;
  }
  else
    lammitys_tot=0;

   // Jos käydään lämpörajoilla, katkaistaan laturin virta. 
  if( volts1 < 1.4 || volts1 >  3.53 )  // 3.533=5C, 1.5=47C, 1.78=40C
    { 
      digitalWrite(CHARGER, HIGH);
      LampoKatkaisu = 1;  // tee tästä varoitus! 
      delay(1000);
      Serial.println("Lampokatkaisu=1, charger HIGH");
    }
  
  if (lastmillis - lampoMillis >= 15000)      
    {
        lampoMillis = lastmillis; 
          if(volts1 < 3.4 && volts1 > 1.6) // 3.38=8C, 1.78=40C
            {
              LampoKatkaisu = 0;
              if(akunjannite < 64)
                {
                  Serial.println("Akku: 48 > x < 65 ");
                  digitalWrite(CHARGER, LOW);  // Lataa
                }
            
              if(akunjannite > 64 && akku_boost == 0)
                {
                  digitalWrite(CHARGER, HIGH);  //sammuta lataus -> tavoite 4.0V kennojännite
                  Serial.println("Akku_boost=0");
                }       
              if(akunjannite < 67 && akku_boost == 1)
                {
                  digitalWrite(CHARGER, LOW);  //käynnistä lataus -> tavoite 4.2V kennojännite
                  Serial.println("alle 68V boost=1");
                }           
            
            }
    Serial.print("Akkuboost: ");        
    Serial.println(akku_boost);
    Serial.print("Boostsensor: ");
    Serial.println(akkuboostsensor);
    akkuboostsensor = akku_boost;
    }



  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
      mqttconnect();
    }
  
  client.loop();


  
  long now = millis();
  if (now - lastMsg > 15000) 
  {
    lastMsg = now;
    janniteviesti = ads.computeVolts(ads.readADC_SingleEnded(3)) * 31.8;
    NTC_viesti = ads.computeVolts(ads.readADC_SingleEnded(2));
    LM35_viesti = ads.computeVolts(ads.readADC_SingleEnded(0)) * 100;
    bttpwr_viesti = Output;
    heatsw_viesti = LampoKatkaisu;
    boost_viesti = akku_boost; 
    lammtot_viesti = lammitys_tot;
    millis_viesti = millis() / 1000;
    bsense_viesti = akkuboostsensor;
    ki_viesti = Ki;
    kp_viesti = Kp;


    /*
    Serial.print("LAmpokatkaisu-mainloop: ");
    Serial.println(LampoKatkaisu);

    Serial.print("Output-mainloop: ");
    Serial.println(Output);

  Serial.print("Input: ");
  Serial.println(Input);
  
  Serial.print("Output: ");
  Serial.println(Output);
  
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);
  */
        
    if (!isnan(janniteviesti) && !isnan(NTC_viesti) && !isnan(LM35_viesti))

    {

      Serial.print("HeatSW: ");
      Serial.println(heatsw_viesti);

      Serial.print("Tehot: ");
      Serial.println(bttpwr_viesti);
      
      snprintf(v_msg, 8, "%f", janniteviesti);
      snprintf(ntc_msg, 8, "%f", NTC_viesti);
      snprintf(lm35_msg, 8, "%f", LM35_viesti);
      snprintf(bttpwr_msg, 4, "%f", bttpwr_viesti);
      snprintf(heatsw_msg, 4, "%f", heatsw_viesti);
      snprintf(boostsens_msg , 4, "%f", bsense_viesti);
      snprintf(lammtotmsg, 4, "%f", lammtot_viesti);
      snprintf(ms_msg, 8, "%f", millis_viesti);
      snprintf(ki_msg, 8, "%f", ki_viesti);
      snprintf(kp_msg, 8, "%f", kp_viesti);


      /* publish the message */
      client.publish(AKKUJANNITE, v_msg);
      client.publish(AKKULAMPO_NTC, ntc_msg);
      client.publish(AKKULAMPO_LM35, lm35_msg);
      client.publish(AKKULAMPOTEHOT, bttpwr_msg);
      client.publish(LAMPOKATKAISU, heatsw_msg);
      client.publish(UPTIMES, ms_msg);
      client.publish(LAMMTOT, lammtotmsg);
      client.publish(BOOSTSENSE, boostsens_msg);
      client.publish(KP, kp_msg);
      client.publish(KI, ki_msg);

      Serial.println(" ");
      //delay(100);
      }
  }

//uusvanhakoodi

  
  




  

}