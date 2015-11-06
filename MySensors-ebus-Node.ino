#include <MyHwATMega328.h>
#include <MySensor.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "crc8.h"
#include <Time.h>

/*
  Vaillant energy bus sensor node


 */

#define NODE_ID       0xB5
#define NODE_TEXT     "VaillantEnergyBus"
#define NODE_VERSION  "0.8"

#define RECEIVE_PIN   A0
#define TRANSMIT_PIN  11
#define REFRESH_INTERVAL 60000 //3600000  // Every hour

// VTT wordt 90 als de ketel aanslaat.
// VT en NT gaan dan oplopen
#define SENSOR_VTT 1
#define SENSOR_VT 3
#define SENSOR_NT 4
#define SENSOR_WTT 5
// WT = Brauchwasser Auslauftemperatur (Water Temperature)
#define SENSOR_WT 6
#define SENSOR_STT 7
// ST = Brauchwasser Speichertemperatur (Boiler Temperature)
#define SENSOR_ST 8
#define SENSOR_HEATING 9
#define SENSOR_HOT_WATER 10
#define SENSOR_MODEL 11
#define SENSOR_HOT_WATER_PUMP 12
#define SENSOR_PUMP 13
#define SENSOR_EBUS 14
#define SENSOR_STATE 15
// TA = Outside temperature
#define SENSOR_TA 16 
#define SENSOR_DT 17

class CSensor
{
private:
  unsigned long m_Millis;
protected:
  const byte    m_ID;
  const char  * m_Name;
public:
  CSensor(byte a_ID, const char * a_Name);
  bool NeedsRefresh();
  void Touch();
};

class CFloatSensor: public CSensor
{
private:
  float m_Value;
public:
  CFloatSensor(byte a_ID, const char * a_Name);
  bool SetValue(float a_Value);
  float Value();
};

float ProcessData1c(const byte a_Offset, CFloatSensor & a_Sensor);
float ProcessData2b(const byte a_Offset, CFloatSensor & a_Sensor);
float ProcessData2c(const byte a_Offset, CFloatSensor & a_Sensor);

SoftwareSerial mySerial(RECEIVE_PIN, TRANSMIT_PIN); // RX, TX
// NRFRF24L01 radio driver (set low transmit power by default)
MyTransportNRF24 radio(RF24_CE_PIN, RF24_CS_PIN, RF24_PA_LEVEL_GW);
// Select AtMega328 hardware profile
MyHwATMega328 hw;
// Construct MySensors library
MySensor gw(radio, hw);
// Initialize temperature message

int packetBytes = 0;
byte packet[128];

bool heating = false;
bool water = false;
byte pump = 0;
byte hotWaterPump = false;
time_t ebusTime = 0;
time_t controllerTime = 0;

CFloatSensor vtt(SENSOR_VTT, "VTT");
CFloatSensor vt(SENSOR_VT, "VT");
CFloatSensor nt(SENSOR_NT, "NT");
CFloatSensor stt(SENSOR_STT, "STT");
CFloatSensor st(SENSOR_ST, "ST");
CFloatSensor wtt(SENSOR_WTT, "WTT");
CFloatSensor wt(SENSOR_WT, "WTT");
CFloatSensor ta(SENSOR_TA, "TA");

void setup()
{
  setSyncProvider( RequestSync);  //set function to call when sync required
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);

  // Initialize library and add callback for incoming messages
  gw.begin(NULL, NODE_ID, false);
  Serial.println(F(NODE_TEXT  " "  NODE_VERSION));
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo(NODE_TEXT, NODE_VERSION);
  gw.present(SENSOR_VT, S_TEMP, "VT");
  gw.present(SENSOR_VTT, S_TEMP, "VTT");
  gw.present(SENSOR_NT, S_TEMP, "NT");
  gw.present(SENSOR_WTT, S_TEMP, "WTT");
  gw.present(SENSOR_WT, S_TEMP, "WT");
  gw.present(SENSOR_STT, S_TEMP, "STT");
  gw.present(SENSOR_ST, S_TEMP, "ST");
  gw.present(SENSOR_HEATING, S_BINARY);
  gw.present(SENSOR_HOT_WATER, S_BINARY);
  gw.present(SENSOR_MODEL, S_CUSTOM);
  gw.present(SENSOR_EBUS, S_CUSTOM);
  gw.present(SENSOR_STATE, S_CUSTOM);
  gw.present(SENSOR_DT, S_CUSTOM);
}

float ProcessData2b(const byte a_Offset, CFloatSensor & a_Sensor)
{
  float value;
  if ((packet[a_Offset+1] & 0x80) == 0x80)
  {
    value = -packet[a_Offset+1] + ((packet[a_Offset] + 1.0) / 256);
  }
  else
  {
    value = packet[a_Offset+1] + ((packet[a_Offset] + 1.0) / 256);
  }
  a_Sensor.SetValue(value);
}

float ProcessData2c(const byte a_Offset, CFloatSensor & a_Sensor)
{
  float value;
  if ((packet[a_Offset+1] & 0x80) == 0x80)
  {
    value = -packet[a_Offset+1] + ((packet[a_Offset] + 1.0) / 256);
  }
  else
  {
    value = packet[a_Offset+1] + ((packet[a_Offset] + 1.0) / 256);
  }
  a_Sensor.SetValue(value);
}

float ProcessData1c(const byte a_Offset, CFloatSensor & a_Sensor)
{
  if (a_Sensor.SetValue(packet[a_Offset] / 2.0))
  {
    /*
    Serial.print(name);
    Serial.print("=");
    Serial.print(current);
    Serial.print("->");
    Serial.print(v);
    Serial.print(" C");
    Serial.println();
    */
    ShowValues();
  }
}

bool ProcessDataBit(byte value, byte bit, bool & current, int sensor, char * name)
{
  bool v = ((1 << bit) & value) != 0;
  //bool v = ((value >> bit) & 0x1) == 0x1;
  if (v != current)
  {
    Serial.print(name);
    Serial.print("=");
    Serial.print(v);
    Serial.println();
    current = v;
    ShowValues();
    // Send in the new binary
    MyMessage msg(sensor, V_STATUS);
    gw.send(msg.set(current));
  }
}

byte ProcessDataByte(byte value, byte & current, int sensor, char * name)
{
  if (value != current)
  {
    Serial.print(name);
    Serial.print("=");
    Serial.print(value);
    Serial.println();
    current = value;
    ShowValues();
    // Send in the new binary
    MyMessage msg(sensor, V_STATUS);
    gw.send(msg.set(current));
  }
}

void ParseVaillantTelegram();
void ReconstructTelegram();

time_t RequestSync()
{
  // Request the time from the controller
  gw.requestTime(ControllerTime);
  return 0; // the time will be sent later in response to serial mesg
}

void ControllerTime(unsigned long a_SecondsSince1970)
{
  controllerTime = a_SecondsSince1970;
}

void loop() // run over and over
{
  // Alway process incoming messages whenever possible
  gw.process();
  if (mySerial.available())
  {
    int data = mySerial.read();
    if (data != 0xAA) 
    {
      packet[packetBytes] = data;
      packetBytes++;
      Serial.print(data, HEX);
      Serial.print(" ");
    }
    else 
    {
      // SYN
      if (packetBytes > 0)
      {
        Serial.println("");
      }
      if (packetBytes >= 5)
      {
        byte length = packet[4];
        if (packetBytes < 6 + length)
        {
          Serial.println(F("Not enough bytes!"));
          packetBytes = 0;
          return;
        }
        byte crc = packet[5 + length];
        byte * data = &packet[0];
        if (CalculateCRC(data, 5 + length) != crc)
        {
          Serial.println(F("CRC error"));
          packetBytes = 0;
          return;
        }
        ReconstructTelegram();
        /*
        MyMessage msg(SENSOR_EBUS, V_VAR2);
        gw.send(msg.set((void*)packet, min(packetBytes, MAX_PAYLOAD)));
        */
        if (packet[2] == 0xB5)
        {
          ParseVaillantTelegram();
        }
        else if (packet[2] == 0x07)
        {
          if (packetBytes >= 20)
          {
            Serial.print(F("Identification: "));
            char model[MAX_PAYLOAD];
            strcpy(model, "Vaillant ");
            strncat(model, (char*)&packet[9], 5);
            sprintf(&model[strlen(model)], " HW-%x.%x SW-%x.%x", packet[16], packet[17], packet[14], packet[15]);
            Serial.println(model);
            /*
            MyMessage msg(SENSOR_MODEL, V_VAR2);
            gw.send(msg.set(model));
            */
          }
        }
        else 
        {
          Serial.println(F("Unhandled telegram"));
        }
        packetBytes = 0;
      }
    }
  }
}

static inline byte Bcd2Dec(byte hex)
{
  return ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
}

void ParseVaillantTelegram()
{
  if (packet[3] == 0x04)
  {
    if (packet[5] == 0x01)
    {
      if (packetBytes >= 11)
      {
          Serial.print(F("Vaillant Get Operating Mode: TV")); 
          Serial.print(packet[9], HEX);
          Serial.print(F(" op.mode="));
          Serial.println(packet[10], HEX);
      }
    }
    else if (packet[5] == 0x00)
    {
      Serial.println(F("Vaillant Get Operational Data - DCF date/time"));
    }
    else
    {
      Serial.println(F("Vaillant Get Operational Data - unhandled block"));
    }
  }
  else if (packet[3] == 0x10)
  {
    Serial.println(F("Vaillant Room Controller: ")); 
    if (packetBytes >= 16)
    {
      ProcessData1c(7, vtt);
      ProcessData1c(8, stt);
    }
  }
  else if (packet[3] == 0x11)
  {
    if (packet[5] == 0x01)
    {
      Serial.println(F("Vaillant Burner Control Unit - block 1")); 
      ProcessData1c(9, vt);
      ProcessData1c(10, nt);
      ProcessData2b(11, ta);
      ProcessData1c(13, wt);
      ProcessData1c(14, st);
      ProcessDataBit(packet[15], 0, heating, SENSOR_HEATING, "H");
      ProcessDataBit(packet[15], 1, water, SENSOR_HOT_WATER, "W");
    }
    else if (packet[5] == 0x02)
    {
      Serial.println(F("Vaillant Burner Control Unit - block 2")); 
      ProcessData1c(13, wtt);
    }
    else
    {
      Serial.println(F("Vaillant Burner Control Unit - unhandled block"));
    }
  }
  else if (packet[3] == 0x12)
  {
    Serial.println(F("Vaillant pump"));
    if ((packet[0] == 0x10) && (packet[1] == 0x08))
    {
      // Heater controller
      //ProcessDataBit(packet[6], 7, hwcp, SENSOR_HWCP, "HWCP");
      if (packet[5] == 0x00)
      {
        switch (packet[6])
        {
        case 0x00: 
          //Serial.println("hot water circulating pump is off"); 
          ProcessDataByte(0, hotWaterPump, SENSOR_HOT_WATER_PUMP, "HWP");
          break;
        case 0x64: 
          //Serial.println("hot water circulating pump is on"); 
          ProcessDataByte(1, hotWaterPump, SENSOR_HOT_WATER_PUMP, "HWP");
          break;
        }
      }
    }
    if ((packet[0] == 0x03) && (packet[1] == 0x64))
    {
      if (packet[5] == 0x02)
      {
        switch (packet[6])
        {
        case 0x00: 
          //Serial.println("internal pump is off"); 
          ProcessDataByte(0, pump, SENSOR_PUMP, "P");
          break;
        case 0x64:
          //Serial.println("internal pump is operating in the service water circuit"); 
          ProcessDataByte(1, pump, SENSOR_PUMP, "P");
          break;
        case 0xFE: 
          //Serial.println("internal pump is operating in the heating circuit"); 
          ProcessDataByte(2, pump, SENSOR_PUMP, "P");
          break;
        }
      }
    }
  }
  else if (packet[3] == 0x16)
  {
    if (packet[5] == 0x00)
    {
      Serial.print(F("Vaillant Broadcast Service - date / time "));
      char buffer[20] = {0}; 
      snprintf(buffer, sizeof(buffer), "20%02x-%02x-%02x %02x:%02x:%02x", packet[12], packet[10], packet[9], packet[8], packet[11], packet[7]);
      Serial.println(buffer); 
      tmElements_t t;
      t.Year = 2000 + Bcd2Dec(packet[12]);
      t.Month = Bcd2Dec(packet[10]);
      t.Day = Bcd2Dec(packet[9]);
      t.Hour = Bcd2Dec(packet[8]);
      t.Minute = Bcd2Dec(packet[11]);
      t.Second = Bcd2Dec(packet[7]);
      time_t unix = makeTime(t);
      /* low level functions to convert to and from system time                     */
/*void breakTime(time_t time, tmElements_t &tm);  // break time_t into elements
time_t makeTime(tmElements_t &tm);  // convert time elements into time_t*/
      
      MyMessage msg(SENSOR_DT, V_VAR2);
      gw.send(msg.set(buffer));
      
    }
    else if (packet[5] == 0x01)
    {
      Serial.println(F("Vaillant broadcast service"));
    }
  }
  else
  {
    Serial.println(F("unhandled Vaillant telegram"));
  }
}

void ReconstructTelegram()
{
  for (int i=0;i<packetBytes;i++)
  {
    if (packet[i] == 0xA9)
    {
      if (packet[i+1] == 0x01)
      {
        packet[i] = 0xAA;
      }
      for (int j=i+1; j<packetBytes-1;j++)
      {
        packet[j] = packet[j+1];
      }
    }
  }
}

void ShowValues()
{
  Serial.print("\tVT=");
  Serial.print(vtt.Value(), 1);
  Serial.print("(");
  Serial.print(vtt.Value(),1);
  Serial.print(") NT=");
  Serial.print(nt.Value(),1);
  Serial.print(" WT=");
  Serial.print(wt.Value(),1);
  Serial.print("(");
  Serial.print(wtt.Value(),1);
  Serial.print(") ST=");
  Serial.print(st.Value(),1);
  Serial.print("(");
  Serial.print(stt.Value(),1);
  Serial.print(")\tH=");
  Serial.print(heating);
  Serial.print(" W=");
  Serial.print(water);
  Serial.print("\tP=");
  Serial.print(pump);
  Serial.print(" HWP=");
  Serial.print(hotWaterPump);
  Serial.println("");
}


CSensor::CSensor(byte a_ID, const char * a_Name) :
  m_ID(a_ID),
  m_Name(a_Name)
{
}

bool CSensor::NeedsRefresh()
{
  unsigned long now = millis();
  if ((m_Millis - now) > REFRESH_INTERVAL)
  {
    return true;
  };
  return false;
}

void CSensor::Touch()
{
  m_Millis = millis();
}

CFloatSensor::CFloatSensor(byte a_ID, const char * a_Name) :
  CSensor(a_ID, a_Name)
{
  m_Value = 0;
}

bool CFloatSensor::SetValue(float a_Value)
{
  if ((a_Value != m_Value) || NeedsRefresh())
  {
    m_Value = a_Value;
    // Send in the new temperature
    MyMessage msg(m_ID, V_TEMP);
    gw.send(msg.set(m_Value, 1));
    Touch();
    return true;
  }
  return false;
}

float CFloatSensor::Value()
{
  return m_Value; 
}

/*
Weergave  Betekenis
CV-bedrijf
S.00  CV geen warmtevraag
S.01  CV-functie ventilator start
S.02  CV-functie pomp voorloop
S.03  CV-bedrijf ontsteking
S.04  CV-functie brander aan
S.05  CV-functie pomp-/ventilator naloop
S.06  CV-functie ventilator naloop
S.07  CV-functie pomp naloop
S.08  CV-functie branderwachttijd
Warmwaterfunctie
S.10  Warmwateraanvraag door stromingssensor
S.11  Warmwaterfunctie ventilatorstart
S.13  Warmwaterfunctie ontsteking
S.14  Warmwaterfunctie brander aan
S.15  Warmwaterfunctie pomp-/ventilator naloop
S.16  Warmwaterfunctie ventilatornaloop
S.17  Warmwaterfunctie pomp naloop
Comfortmodus warme start of warmwatermodus met actoSTOR
S.20  Warmwatervraag
S.21  Warmwaterfunctie ventilatorstart
S.22  Warmwaterfunctie pomp voorloop
S.23  Warmwaterfunctie ontsteking
S.24  Warmwaterfunctie brander aan
S.25  Warmwaterfunctie pomp-/ventilator naloop
S.26  Warmwaterfunctie ventilatornaloop
S.27  Warmwaterfunctie pomp naloop
S.28  Warm water branderwachttijd
Speciale gevallen
S.30  Kamerthermostaat (RT) blokkeert CV vraag
S.31  Zomermodus actief of geen warmtevraag door eBusthermostaat
S.32  Wachttijd wegens afwijking ventilatortoerental
S.34  Vorstbeveiligingsfunctie actief
S.39  "Brander uit contact" is geactiveerd (bijv. aanlegthermostaat of condensaatpomp)
S.40  Comfortbeveiligingsmodus is actief: CV-ketel loopt met beperkt verwarmingscomfort (¬ hfdst. 13.2.3)
S.41  Waterdruk > 2,8 bar
S.42  Bevestigingssignaal rookgaskleppen blokkeert branderfunctie (alleen in combinatie met toebehoren VR40) of condensaatpomp defect, warmtevraag wordt geblokkeerd
S.46  Comfortbeveiligingsmodus vlamverlies minimumlast
S.53  CV-ketel bevindt zich in de wachttijd van de modulatieblokkering/blokkeringsfunctie op grond van watergebrek (spreiding aanvoer-retour te groot).
S.54  CV-ketel bevindt zich in de wachttijd van de blokkeringsfunctie op grond van watergebrek (temperatuurgradiënt)
S.57  Wachttijd comfortbeveiligingsmodus
S.58  Modulatiebegrenzing wegens geluidsvorming/wind
S.61  Gasfamiliecontrole niet succesvol: codeerweerstand op de printplaat past niet bij de ingevoerde gasgroep (zie ook F.92).
S.62  Gasfamiliecontrole niet succesvol: CO/CO2-waarden bij grenswaarden. Verbranding controleren.
S.63  Gasfamiliecontrole niet succesvol: verbrandingskwaliteit buiten het toegestane bereik (zie ook F.93).
S.76  Installatiedruk te gering; water bijvullen
S.96  Test NTC-voeler retour loopt, warmtebragen zijn geblokkeerd
S.97  Waterdruksensortest loopt, warmtevragen zijn geblokkeerd
S.98  Test NTC-voeler aanvoer/retour loopt, warmtevragen zijn geblokkeerd
*/
