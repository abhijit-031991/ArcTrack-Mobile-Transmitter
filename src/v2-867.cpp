#define ARDUINOJSON_USE_LONG_LONG 1
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <ArduinoJson.h>

#define SCK     18    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    23   // GPIO27 -- SX1278's MOSI
#define SS      5   // GPIO18 -- SX1278's CS
#define RST     21   // GPIO14 -- SX1278's RESET
#define DI0     22   // GPIO26 -- SX1278's IRQ(Interrupt Request)

BluetoothSerial SerialBT;
byte led = 12;
bool ledStat = false;
unsigned long x = 1;

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
// Callback function implementation
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    ledStat = true;
  }
 
  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");
    ledStat = false;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.register_callback(callback);
  SerialBT.begin("Arc-Track-10112");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);

  WiFi.mode(WIFI_OFF);
  
  if (!LoRa.begin(867E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(41.7E3);
  // LoRa.setGain(6);
  Serial.println("SYSTEM READY");
  // LoRa.onReceive(onReceive);  
pinMode(led, OUTPUT);
    
}

void loop(){
  while (SerialBT.available())
  {
    String dat;
    dat = SerialBT.readString();
    Serial.println(dat);
    DynamicJsonDocument doc(350);
    deserializeJson(doc, dat);    
    if (dat.length() > 0 && dat.length() <10)
    {
      int sensorValue = analogRead(33);
      float voltage = (sensorValue * 3.3 ) / (4095);
      SerialBT.print(voltage*2); SerialBT.println(" V");
      Serial.print(voltage); Serial.println(" V");
    }
    
    if (dat.length() < 35)
    {
      struct req{
      uint16_t tag;
      byte req;
    } __attribute__((__packed__)) r;
    r.tag = doc["ID"];
    r.req = doc["MSG"];

    Serial.print(r.tag);
    Serial.print(r.req);

    LoRa.beginPacket();
    LoRa.write((uint8_t*)&r, sizeof(r));
    LoRa.endPacket();
    }
    if (dat.length() > 195)
    {
      struct setttings{
        uint32_t pingTime;
        uint16_t act_trsh;
        uint16_t act_gps_frq;
        uint16_t act_duration;
        uint16_t gpsFrq;
        uint16_t gpsTout;
        uint8_t hdop;
        uint8_t radioFrq;
        uint8_t rcv_dur;
        uint8_t sch_dur;
        uint8_t sch_rpt_dur;
        bool act_enabled;
        bool sch_enabled;
      } __attribute__((__packed__)) set;


        JsonObject obj = doc.as<JsonObject>();

        if (obj.containsKey("schen"))
        {
          Serial.print("schen");
          Serial.println((bool)obj[F("schen")]);
          set.sch_enabled = (bool)obj[F("schen")];
        }
        if (obj.containsKey("schTime"))
        {
          // unsigned long long x  = obj[F("schTime")];
          Serial.print("schTime");
          Serial.println((uint32_t)obj[F("schTime")]);
          set.pingTime = (uint32_t)obj[F("schTime")];
        }
        if (obj.containsKey("schDur"))
        {
          Serial.print("schDur");
          Serial.println((uint8_t)obj[F("schDur")]);
          set.sch_dur = (uint8_t)obj[F("schDur")];
        }
        if (obj.containsKey("schRpt"))
        {
          Serial.print("schRpt");
          Serial.println((uint8_t)obj[F("schRpt")]);
          set.sch_rpt_dur = (uint8_t)obj[F("schRpt")];
        }
        if (obj.containsKey("acten"))
        {
          Serial.print("acten");
          Serial.println((bool)obj[F("acten")]);
          set.act_enabled = (bool)obj[F("acten")];
        }
        if (obj.containsKey("actTrsh"))
        {
          Serial.print("actTrsh");
          Serial.println((uint16_t)obj[F("actTrsh")]);
          set.act_trsh = (uint16_t)obj[F("actTrsh")];
        }
        if (obj.containsKey("actDur"))
        {
          Serial.print("actDur");
          Serial.println((uint16_t)obj[F("actDur")]);
          set.act_duration = (uint16_t)obj[F("actDur")];
        }
        if (obj.containsKey("gpsHres"))
        {
          Serial.print("gpsHres");
          Serial.println((uint16_t)obj[F("gpsHres")]);
          set.act_gps_frq = (uint16_t)obj[F("gpsHres")];
        }
        if (obj.containsKey("gpsLres"))
        {
          Serial.print("gpsLres");
          Serial.println((uint16_t)obj[F("gpsLres")]);
          set.gpsFrq = (uint16_t)obj[F("gpsLres")];
        }
        if (obj.containsKey("gpsTimeout"))
        {
          Serial.print("gpsTimeout");
          Serial.println((uint16_t)obj[F("gpsTimeout")]);
          set.gpsTout = (uint16_t)obj[F("gpsTimeout")];
        }
        if (obj.containsKey("gpsHdop"))
        {
          Serial.print("gpsHdop");
          Serial.println((uint8_t)obj[F("gpsHdop")]);
          set.hdop = (uint8_t)obj[F("gpsHdop")];
        }
        if (obj.containsKey("PngIntN"))
        {
          Serial.print("PngIntN");
          Serial.println((uint8_t)obj[F("PngIntN")]);
          set.radioFrq = (uint8_t)obj[F("PngIntN")];
        }
        if (obj.containsKey("rcvDur"))
        {
          Serial.print("rcvDur");
          Serial.println((uint8_t)obj[F("rcvDur")]);
          set.rcv_dur = (uint8_t)obj[F("rcvDur")];
        }

        Serial.println(set.pingTime);
        Serial.println(set.act_trsh);
        Serial.println(set.act_gps_frq);
        Serial.println(set.act_duration);
        Serial.println(set.gpsFrq);
        Serial.println(set.gpsTout);
        Serial.println(set.hdop);
        Serial.println(set.radioFrq);
        Serial.println(set.rcv_dur);
        Serial.println(set.sch_dur);
        Serial.println(set.sch_rpt_dur);
        Serial.println(set.act_enabled);
        Serial.println(set.sch_enabled);

        
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&set, sizeof(set));
        LoRa.endPacket();
    }
    
    if (dat.length() > 90 && dat.length() < 135)
    {
      struct setttings{
        uint16_t id;
        uint16_t wTrsh;
        uint16_t aTrsh;
        uint16_t gpsFrq;
        uint16_t gpsTout;
        uint8_t hdop;
        uint8_t radioFrq;
        uint8_t rcv_dur;
        uint8_t mprFrq;
      } __attribute__((__packed__)) set;

      JsonObject obj = doc.as<JsonObject>();

        if (obj.containsKey("id"))
        {
          Serial.print("id");
          Serial.println((uint16_t)obj[F("id")]);
          set.mprFrq = (uint16_t)obj[F("id")];
        }
        if (obj.containsKey("mprFrq"))
        {
          Serial.print("mprFrq");
          Serial.println((uint8_t)obj[F("mprFrq")]);
          set.mprFrq = (uint8_t)obj[F("mprFrq")];
        }
        if (obj.containsKey("airTrsh"))
        {
          Serial.print("airTrsh");
          Serial.println((uint16_t)obj[F("airTrsh")]);
          set.aTrsh = (uint16_t)obj[F("airTrsh")];
        }
        if (obj.containsKey("wtrTrsh"))
        {
          Serial.print("wtrTrsh");
          Serial.println((uint16_t)obj[F("wtrTrsh")]);
          set.wTrsh = (uint16_t)obj[F("wtrTrsh")];
        }
        if (obj.containsKey("gpsHres"))
        {
          Serial.print("gpsHres");
          Serial.println((uint16_t)obj[F("gpsHres")]);
          set.gpsFrq = (uint16_t)obj[F("gpsHres")];
        }
        if (obj.containsKey("gpsTimeout"))
        {
          Serial.print("gpsTimeout");
          Serial.println((uint16_t)obj[F("gpsTimeout")]);
          set.gpsTout = (uint16_t)obj[F("gpsTimeout")];
        }
        if (obj.containsKey("gpsHdop"))
        {
          Serial.print("gpsHdop");
          Serial.println((uint8_t)obj[F("gpsHdop")]);
          set.hdop = (uint8_t)obj[F("gpsHdop")];
        }
        if (obj.containsKey("PngIntN"))
        {
          Serial.print("PngIntN");
          Serial.println((uint8_t)obj[F("PngIntN")]);
          set.radioFrq = (uint8_t)obj[F("PngIntN")];
        }
        if (obj.containsKey("rcvDur"))
        {
          Serial.print("rcvDur");
          Serial.println((uint8_t)obj[F("rcvDur")]);
          set.rcv_dur = (uint8_t)obj[F("rcvDur")];
        }

        Serial.println(set.gpsFrq);
        Serial.println(set.gpsTout);
        Serial.println(set.hdop);
        Serial.println(set.radioFrq);
        Serial.println(set.rcv_dur);
        Serial.println(set.mprFrq);
        Serial.println(set.wTrsh);
        Serial.println(set.aTrsh);

        
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&set, sizeof(set));
        LoRa.endPacket();


    }
    
  }

  int x = LoRa.parsePacket();
  if (x != 0)
  {
    Serial.println(x);
  }
  
  
  if (x == 15) /// DevType 101 - Data
  {
    String dat;
    struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    byte hdop;
    // byte id;
    } __attribute__((__packed__)) d;

    while (LoRa.available())
      {
        LoRa.readBytes((uint8_t*)&d, sizeof(d));
      }
      StaticJsonDocument<128> doc;
      doc[F("Date")] = d.datetime;
      doc[F("Lat")] = d.lat;
      doc[F("Lng")] = d.lng;
      doc[F("hdop")] = d.hdop;
      doc[F("lktm")] = d.locktime;
      doc[F("RSSI")] = LoRa.packetRssi();
      serializeJson(doc, dat);
      SerialBT.println(dat);
      Serial.println(dat);
  }
  if (x == 20) /// DevType 105 - Data
  {
    String dat;
    struct data
    {
      uint32_t datetime;
      float x;
      float y;
      float z;
      float od; 
    }d; 
    // __attribute__((__packed__)) d;

    while (LoRa.available())
      {
        LoRa.readBytes((uint8_t*)&d, sizeof(d));
      }
      StaticJsonDocument<128> doc;
      doc[F("Date")] = d.datetime;
      doc[F("X")] = d.x;
      doc[F("Y")] = d.y;
      doc[F("Z")] = d.z;
      doc[F("ODBA")] = d.od;
      serializeJson(doc, dat);
      SerialBT.println(dat);
      Serial.println(dat);
  }
  if (x == 3) /// Universal Ping - Request/Response
  {
    String dat;
    struct resp{
      uint16_t tag;
      byte res;
    }r;

    while (LoRa.available())
    {
      LoRa.readBytes((uint8_t*)&r, x);
    }
    StaticJsonDocument<128> doc;
    doc[F("ID")] = r.tag;
    doc[F("Msg")] = r.res;
    doc[F("RSSI")] = LoRa.packetRssi();
    serializeJson(doc, dat);

    SerialBT.println(dat);
    Serial.println(dat);
  }
  if (x == 13) /// DevType 101/105/107 - Ping
  {
    Serial.println(F("Received Ping"));
    String dat;
    struct ping{
      uint16_t ta;
      uint16_t cnt;
      float la;
      float ln;
      uint8_t devtyp;
    } __attribute__((__packed__)) p;

    while (LoRa.available())
    {
      LoRa.readBytes((uint8_t*)&p, x);
    }

    StaticJsonDocument<256> doc;
    doc[F("ID")] = p.ta;
    doc[F("Lat")] = String(p.la, 6);
    doc[F("Lng")] = String(p.ln, 6);
    doc[F("DTyp")] = p.devtyp;
    doc[F("cnt")] = p.cnt;
    doc[F("RSSI")] = LoRa.packetRssi();
    serializeJson(doc, dat);
    SerialBT.println(dat);
    Serial.println(dat);
  }  
  if (x == 35) /// DevType 105 - Data
  {
    String dat;
    struct data{
        uint32_t datetime;
        uint16_t locktime;
        float lat;
        float lng;
        float t;
        float p;
        float x;
        float y;
        float z;
        byte hdop;
    }__attribute__((__packed__)) d;

    while (LoRa.available())
      {
        LoRa.readBytes((uint8_t*)&d, sizeof(d));
      }
      StaticJsonDocument<256> doc;
      doc[F("Date")] = d.datetime;
      doc[F("Lat")] = d.lat;
      doc[F("Lng")] = d.lng;
      doc[F("LckTm")] = d.locktime;
      doc[F("hdop")] = d.hdop;
      doc[F("Temp")] = d.t;
      doc[F("Pres")] = d.p;
      doc[F("x")] = d.x;
      doc[F("y")] = d.y;
      doc[F("z")] = d.z;
      
      serializeJson(doc, dat);
      SerialBT.println(dat);
      Serial.println(dat);
  }
  if (x == 16) /// DevType 107 - Data
    {
      String dat;
      struct data{
          uint32_t datetime;
          uint16_t locktime;
          float lat;
          float lng;
          byte hdop;
          bool act;
      }__attribute__((__packed__)) d;

      while (LoRa.available())
        {
          LoRa.readBytes((uint8_t*)&d, sizeof(d));
        }
        StaticJsonDocument<256> doc;
        doc[F("Date")] = d.datetime;
        doc[F("Lat")] = d.lat;
        doc[F("Lng")] = d.lng;
        doc[F("LckTm")] = d.locktime;
        doc[F("hdop")] = d.hdop;
        doc[F("Act")] = d.act;
      
        serializeJson(doc, dat);
        SerialBT.println(dat);
        Serial.println(dat);
    }

  if(ledStat == true){
    if(digitalRead(led) == LOW){
      Serial.println("LED ON");
      digitalWrite(led, HIGH);
    }
    
  }else
  {
    digitalWrite(led, !digitalRead(led));
    delay(500);
  }
}

