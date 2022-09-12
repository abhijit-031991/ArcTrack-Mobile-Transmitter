// #include <Arduino.h>
// #include <BluetoothSerial.h>
// #include <WiFi.h>
// #include <SPI.h>
// #include <Wire.h>
// #include <LoRa.h>
// #include <ArduinoJson.h>

// #define SCK     18    // GPIO5  -- SX1278's SCK
// #define MISO    19   // GPIO19 -- SX1278's MISnO
// #define MOSI    23   // GPIO27 -- SX1278's MOSI
// #define SS      5   // GPIO18 -- SX1278's CS
// #define RST     21   // GPIO14 -- SX1278's RESET
// #define DI0     22   // GPIO26 -- SX1278's IRQ(Interrupt Request)

// BluetoothSerial SerialBT;


// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   SerialBT.begin("Arc-TraCK");
//   SPI.begin(SCK,MISO,MOSI,SS);
//   LoRa.setPins(SS,RST,DI0);

//   WiFi.mode(WIFI_OFF);
  
//   if (!LoRa.begin(433E6)) {
//     Serial.println("Starting LoRa failed!");
//     while (1);
//   }
// //   LoRa.setSpreadingFactor(7);
//   Serial.println("SYSTEM READY");
//   // LoRa.onReceive(onReceive);  
    
// }

// // void loop(){
// //   while (SerialBT.available())
// //   {
// //     String dat;
// //     dat = SerialBT.readString();
// //     Serial.println(dat);
// //     DynamicJsonDocument doc(256);
// //     deserializeJson(doc, dat);    
// //     if (dat.length() < 25)
// //     {
// //       struct req{
// //       byte tag;
// //       byte req;
// //     }r;
// //     r.tag = doc["ID"];
// //     r.req = doc["MSG"];

// //     Serial.print(r.tag);
// //     Serial.print(r.req);

// //     LoRa.beginPacket();
// //     LoRa.write((uint8_t*)&r, sizeof(r));
// //     LoRa.endPacket();
// //     }
// //     if (dat.length() > 25)
// //     {
// //       struct setting{
// //         byte gto;
// //         byte hdop;
// //         byte pfrq;
// //         byte cpsw;
// //         byte sttm;
// //         byte sptm;
// //         uint16_t gfrq;
// //         }s;
// //         JsonObject obj = doc.as<JsonObject>();
// //         if (obj.containsKey("GTO"))
// //         {
// //           s.gto = obj[F("GTO")];
// //           Serial.println((byte)obj[F("GTO")]);
// //         }
// //         if (obj.containsKey("GFRQ"))
// //         {
// //           s.gfrq = obj[F("GFRQ")];
// //           Serial.println((byte)obj[F("GFRQ")]);
// //         }
// //         if (obj.containsKey("HDOP"))
// //         {
// //           s.hdop = obj[F("HDOP")];
// //           Serial.println((byte)obj[F("HDOP")]);
// //         }
// //         if (obj.containsKey("PFRQ"))
// //         {
// //           s.pfrq = obj[F("PFRQ")];
// //           Serial.println((byte)obj[F("PFRQ")]);
// //         }
// //         if (obj.containsKey("CPSW"))
// //         {
// //           s.cpsw = obj[F("CPSW")];
// //           Serial.println((byte)obj[F("CPSW")]);
// //         }
// //         if (obj.containsKey("STTM"))
// //         {
// //           s.sttm = obj[F("STTM")];
// //           Serial.println((byte)obj[F("STTM")]);
// //         }
// //         if (obj.containsKey("SPTM"))
// //         {
// //           s.sptm = obj[F("SPTM")];
// //           Serial.println((byte)obj[F("SPTM")]);
// //         }
        
// //         // s.gfrq = doc[F("GFRQ")];
// //         // s.hdop = doc[F("HDOP")];
// //         // s.pfrq = doc[F("PFRQ")];
// //         // s.cpsw = doc[F("CPSW")];
// //         // s.sttm = doc[F("STTM")];
// //         // s.sptm = doc[F("SPTM")];
// //         Serial.println(s.gto);
// //         Serial.println(s.gfrq);
// //         Serial.println(s.hdop);
// //         Serial.println(s.pfrq);
// //         Serial.println(s.cpsw);
// //         Serial.println(s.sttm);
// //         Serial.println(s.sptm);
// //         LoRa.beginPacket();
// //         LoRa.write((uint8_t*)&s, sizeof(s));
// //         LoRa.endPacket();
// //     }
    
// //   }

// //   int x = LoRa.parsePacket();
// //   if (x == 15)
// //   {
// //     String dat;
// //     struct data{
// //     uint32_t datetime;
// //     uint16_t locktime;
// //     float lat;
// //     float lng;
// //     byte hdop;
// //     // byte id;
// //     } __attribute__((__packed__)) d;

// //     while (LoRa.available())
// //       {
// //         LoRa.readBytes((uint8_t*)&d, sizeof(d));
// //       }
// //       StaticJsonDocument<128> doc;
// //       doc[F("Date")] = d.datetime;
// //       doc[F("Lat")] = d.lat;
// //       doc[F("Lng")] = d.lng;
// //       doc[F("hdop")] = d.hdop;
// //       doc[F("lktm")] = d.locktime;
// //       doc[F("RSSI")] = LoRa.packetRssi();
// //       serializeJson(doc, dat);
// //       SerialBT.println(dat);
// //       Serial.println(dat);
// //   }
// //   if (x == 2)
// //   {
// //     String dat;
// //     struct resp{
// //       byte tag;
// //       byte res;
// //     }r;

// //     while (LoRa.available())
// //     {
// //       LoRa.readBytes((uint8_t*)&r, x);
// //     }
// //     StaticJsonDocument<128> doc;
// //     doc[F("ID")] = r.tag;
// //     doc[F("Msg")] = r.res;
// //     doc[F("RSSI")] = LoRa.packetRssi();
// //     serializeJson(doc, dat);

// //     SerialBT.println(dat);
// //     Serial.println(dat);
// //   }
// //   if (x == 12)
// //   {
// //     String dat;
// //     struct ping{
// //       uint8_t ta;
// //       uint8_t devtyp;
// //       uint16_t cnt;
// //       float la;
// //       float ln;
// //     }p;

// //     while (LoRa.available())
// //     {
// //       LoRa.readBytes((uint8_t*)&p, x);
// //     }

// //     StaticJsonDocument<256> doc;
// //     doc[F("ID")] = p.ta;
// //     doc[F("Lat")] = String(p.la, 6);
// //     doc[F("Lng")] = String(p.ln, 6);
// //     doc[F("DTyp")] = p.devtyp;
// //     doc[F("cnt")] = p.cnt;
// //     doc[F("RSSI")] = LoRa.packetRssi();
// //     serializeJson(doc, dat);
// //     SerialBT.println(dat);
// //     Serial.println(dat);
// //   }  
// // }

