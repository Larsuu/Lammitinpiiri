
//#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <QuickPID.h>
#include <BluetoothSerial.h>
#include <esp_task_wdt.h>
#include <math.h>

//#include <WiFi.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const char* ssid = "OpenWrt";
const char* password = "10209997";

#define HEATER 23
#define CHARGER 22
#define PIN_INPUT &volts2
#define WDT_TIMEOUT 60          // Watchdog timeout 60 sekunttia

bool printOrPlotter = 0;  // on(1) monitor, off(0) plotter
float POn = 1.0;          // proportional on Error to Measurement ratio (0.0-1.0), default = 1.0
float DOn = 0.0;          // derivative on Error to Measurement ratio (0.0-1.0), default = 0.0



// Kirjastojen alias
BluetoothSerial SerialBT;
Adafruit_ADS1115 ads;
       

// Muuttujat
char receivedChars;         
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;
int16_t mapval;
int16_t mapvalinv; 
String BT_vastaanotto = "";
unsigned long prevmillis = 0;
unsigned long lampoMillis = 0;
unsigned long interval = 30000;
boolean LampoKatkaisu = 1;
int tehot = 0;
float ntc1tmp = 0;
float NewSetpoint = 0;


// PID muuttujat
float Setpoint, Input, Output;
float Kp = 3, Ki = 5, Kd = 1;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::reverse);



/*
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(100);
  }
  Serial.println(WiFi.localIP());
}
*/

// aloitetaan setup -määrittelyt
void setup(void)
{

//initWiFi();

  esp_task_wdt_init(WDT_TIMEOUT, true);   //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                 //add current thread to WDT watch

  pinMode(23, OUTPUT);                  // Lämmitinvastuksen ohjauspinni
  pinMode(22, OUTPUT);                  // Laturin rele AC
  Wire.begin(27,26);                    // i2c pinnit ADS1115:lle
  Serial.begin(115200);                 

  ledcAttachPin(HEATER, 0);
  ledcSetup(0, 1, 8);
   
  SerialBT.begin("KontioHeat2.0");                                                //Bluetooth device name

  //initialize the variables we're linked to
  Setpoint = 2.5;        // Lämmityksen oletuslämpö jos ei muutettu volts1=20k=2.5V=25C
  //int GetDirection=1;

  //apply PID gains
  myPID.SetTunings(Kp, Ki, Kd);
 
  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);
 
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  if (!ads.begin()) {                                   // ADS:n odotus = !True
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

}  // void setup loppuu.

float temppi(float adctulos1)
  {
    float temp1 = 0;
    adctulos1 = adctulos1 - 10000;
    temp1 = 1/(1/298.15 + 1/3639 * log(adctulos1/10000)) - 273.15;
    return temp1;


  }

void loop(void)
{

unsigned long lastmillis = millis();    // wdt ja muu aikaindeksi

/*
  if ((WiFi.status() != WL_CONNECTED) && (lastmillis - prevmillis >= interval)) 
  {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    prevmillis = lastmillis;
  }
*/

esp_task_wdt_reset();

   if (Serial.available()) 
      {    
        SerialBT.write(Serial.read()); 
      }     

  if (SerialBT.available()) 
      {                 
        BT_vastaanotto = SerialBT.readStringUntil('^M');  // luetaan BT syöte stringiin, ^m asti (BT terminal rivipääte).
        Serial.print("Input is: ");
        Serial.println(BT_vastaanotto);
        NewSetpoint = BT_vastaanotto.toFloat();              // string to PID setpoint float 
        SerialBT.print("Received: ");
        SerialBT.println(NewSetpoint);    
  
    
      if(Setpoint >= 1.78 && Setpoint <= 3.5)
        {
          SerialBT.print("Temperature Setpoint was: ");
          SerialBT.println(Setpoint);
          SerialBT.print("Temperature volts2 is: ");
          SerialBT.println(volts2);
          SerialBT.print("Temperature adc1 is: ");
          SerialBT.println( temppi(adc1) );
          
        }
      else  
        {
         Setpoint = 2.5;
         SerialBT.print("Limit Error, setpoint back to default: ");// write on BT app
         SerialBT.println(Setpoint);
         SerialBT.print("Temperature is: ");// write on BT app
         SerialBT.println(volts2);
         SerialBT.print("Temperature adc1 is: ");
         SerialBT.println( temppi(adc1) );
         Serial.println("Limit Error");//write on serial monitor
        }    
     
  

  } // SerialBT.available loppuu.
  
   
  // ads1115 ADC määrittelyt.

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
      Serial.print("Output1: "); Serial.print(mapval); Serial.print("  "); Serial.println(" ms");
      Serial.print("Output0: "); Serial.print(mapvalinv); Serial.print("  "); Serial.println(" ms");

      // SerialBT.print("NTC0Temp(Nikkeli): ");
      // SerialBT.println(volts0);

      SerialBT.print("NTC1 Temp(thermalpädi): ");
      SerialBT.println(volts1);
      SerialBT.print("NTC1 in degrees: ");
      SerialBT.println( temppi(adc1) );

      SerialBT.print("Lampokatkaisu: ");
      SerialBT.println(LampoKatkaisu);

      SerialBT.print("Main Voltage: ");
      SerialBT.println(volts3);
      
      SerialBT.print("TempLM35: ");
      SerialBT.println(volts2);

      SerialBT.print("LämmitysTehot: ");
      SerialBT.println(tehot);


    }


  Input = volts1;     // LM35:n lämpötulos luetaan PID loopin inputtiin. 
  myPID.Compute();    // PID laskenta.
  
  mapval = map(Output, 0, 255, 0, 1000);  // PID:n 8bit output tulos mapataan millisekunneiksi.
  mapvalinv = 1000 - mapval;              // 1s invertoitu OFF aika. 

  ledcWrite(0, Output);                   // vastaa analogwrite:ä, kanava ja taajuus 8 bittisenä. 

  tehot = (mapval / 10);


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

} // void loop loppuu


