#define MY_NODE_ID       0xB5
#define MY_RADIO_NRF24
#define MY_DEBUG

#include <MySensor.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "crc8.h"
#include <Time.h>

/*
  Vaillant energy bus sensor node


 */

#define NODE_TEXT     "VaillantEnergyBus"
#define NODE_VERSION  "1.1"

#define RECEIVE_PIN   A0
#define TRANSMIT_PIN  11
#define REFRESH_INTERVAL 300000 // Every five minutes

// todo: omschrijven naar HVAC:
// S_HVAC, // Thermostat/HVAC device. V_HVAC_SETPOINT_HEAT, V_HVAC_SETPOINT_COLD, V_HVAC_FLOW_STATE, V_HVAC_FLOW_MODE, V_TEMP

// VTT wordt 90 als de ketel aanslaat.
// VT en NT gaan dan oplopen
#define SENSOR_VT 3
#define SENSOR_NT 4
// WT = Brauchwasser Auslauftemperatur (Water Temperature)
#define SENSOR_WT 6
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
//#define SENSOR_TA 16 
#define SENSOR_DT 17

class CSensor
{
private:
  unsigned long m_Millis;
protected:
  const byte    m_ID;
  const char  * m_Name;
public:
  CSensor(const byte a_ID, const char * a_Name);
  bool NeedsRefresh();
  void Touch();
};

class CFloatSensor: public CSensor
{
private:
  float m_Value;
  byte  m_Type;
public:
  CFloatSensor(const byte a_ID, const byte a_Type, const char * a_Name);
  bool SetValue(const float a_Value);
  float Value() const;
};

class CBitSensor: public CSensor
{
private:
  bool m_Value;
  byte m_Type;
public:
  CBitSensor(const byte a_ID, const byte a_Type, const char * a_Name);
  bool SetValue(const bool a_Value);
  bool Value() const;
};

class CByteSensor: public CSensor
{
private:
  bool m_Value;
  byte m_Type;
public:
  CByteSensor(const byte a_ID, const byte a_Type, const char * a_Name);
  bool SetValue(const byte a_Value);
  byte Value() const;
};

void ProcessData1c(const byte a_Offset, CFloatSensor & a_Sensor);
void ProcessData2b(const byte a_Offset, CFloatSensor & a_Sensor);
void ProcessData2c(const byte a_Offset, CFloatSensor & a_Sensor);
void ProcessDataBit(const byte a_Offset, const byte a_Bit, CBitSensor & a_Sensor);

SoftwareSerial mySerial(RECEIVE_PIN, TRANSMIT_PIN); // RX, TX

int packetBytes = 0;
byte packet[128];

time_t ebusTime = 0;
time_t controllerTime = 0;

CFloatSensor vt (SENSOR_VT, V_TEMP, "VT");
CFloatSensor vtt(SENSOR_VT, V_HVAC_SETPOINT_HEAT, "VTT");
CFloatSensor nt (SENSOR_NT, V_TEMP, "NT");
CFloatSensor st (SENSOR_ST, V_TEMP, "ST");
CFloatSensor stt(SENSOR_ST, V_HVAC_SETPOINT_HEAT, "STT");
CFloatSensor wt (SENSOR_WT, V_TEMP, "WT");
CFloatSensor wtt(SENSOR_WT, V_HVAC_SETPOINT_HEAT, "WTT");
#ifdef SENSOR_TA
  CFloatSensor ta (SENSOR_TA, V_TEMP, "TA");
#endif
CBitSensor heating(SENSOR_HEATING,   V_STATUS, "H");
CBitSensor water  (SENSOR_HOT_WATER, V_STATUS, "W");
CByteSensor pump(SENSOR_PUMP, V_STATUS, "P");
CByteSensor hotWaterPump(SENSOR_HOT_WATER_PUMP, V_STATUS, "HWP");

char state[MAX_PAYLOAD] = {0};

void CheckState()
{
  char newState[MAX_PAYLOAD];
  sprintf(newState, "%d %d.%d %d.%d", vtt.Value() != 0, heating.Value(), water.Value(), pump.Value(), hotWaterPump.Value());
  if (strcmp(state, newState) != 0)
  {
    Serial.print("State change: ");
    strcpy(state, newState);
    Serial.println(state);
    MyMessage msg(SENSOR_STATE, V_TEXT);
    send(msg.set(state));
  }
}

void setup()
{
  // setSyncProvider( RequestSync);  //set function to call when sync required
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
}

void presentation()
{
  Serial.println(F(NODE_TEXT  " "  NODE_VERSION));
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(NODE_TEXT, NODE_VERSION);
  present(SENSOR_VT, S_HEATER, "VT");
  present(SENSOR_NT, S_TEMP, "NT");
  present(SENSOR_WT, S_HEATER, "WT");
  present(SENSOR_ST, S_HEATER, "ST");
  present(SENSOR_HEATING, S_BINARY, "H");
  present(SENSOR_HOT_WATER, S_BINARY, "HW");
  present(SENSOR_MODEL, S_CUSTOM, "MODEL");
  present(SENSOR_EBUS, S_CUSTOM, "EBUS");
  present(SENSOR_STATE, S_CUSTOM, "State");
  present(SENSOR_DT, S_CUSTOM, "DT");
  present(SENSOR_PUMP, S_CUSTOM, "P");
  present(SENSOR_HOT_WATER_PUMP, S_CUSTOM, "HWP");
}

void ProcessData2b(const byte a_Offset, CFloatSensor & a_Sensor)
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

void ProcessData2c(const byte a_Offset, CFloatSensor & a_Sensor)
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

void ProcessData1c(const byte a_Offset, CFloatSensor & a_Sensor)
{
  if (a_Sensor.SetValue(packet[a_Offset] / 2.0))
  {
    ShowValues();
  }
}

void ProcessDataBit(const byte a_Offset, const byte a_Bit, CBitSensor & a_Sensor)
{
  bool v = ((1 << a_Bit) & packet[a_Offset]) != 0;
  if (a_Sensor.SetValue(v))
  {
    ShowValues();
  }
}

void ParseVaillantTelegram();
void ReconstructTelegram();

void receiveTime(unsigned long a_SecondsSince1970)
{
  controllerTime = a_SecondsSince1970;
}

void loop() // run over and over
{
  if (mySerial.available())
  {
    int data = mySerial.read();
    if (data != 0xAA) 
    {
      packet[packetBytes] = data;
      packetBytes++;
      if (data <= 0xF) Serial.print(" ");
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
            sprintf(model, " %x.%x/%x.%x", packet[16], packet[17], packet[14], packet[15]);
            Serial.println(model);
            
            MyMessage msg(SENSOR_MODEL, V_TEXT);
            send(msg.set(model));
            
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

void ParseVaillantTelegram04()
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

void ParseVaillantTelegram09()
{
  Serial.println(F("Vaillant 09: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ")); 
}

void ParseVaillantTelegram10()
{
  Serial.println(F("Vaillant Room Controller: ")); 
  if (packetBytes >= 16)
  {
    ProcessData1c(7, vtt);
    ProcessData1c(8, stt);
    CheckState();
  }
}

void ParseVaillantTelegram11()
{
  if (packet[5] == 0x01)
  {
    Serial.println(F("Vaillant Burner Control Unit - block 1")); 
    ProcessData1c(9, vt);
    ProcessData1c(10, nt);
#ifdef SENSOR_TA      
    ProcessData2b(11, ta);
#endif
    ProcessData1c(13, wt);
    ProcessData1c(14, st);
    ProcessDataBit(packet[15], 0, heating);
    ProcessDataBit(packet[15], 1, water);
    CheckState();
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

void ParseVaillantTelegram12()
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
        if (hotWaterPump.SetValue(0)) CheckState();
        break;
      case 0x64: 
        //Serial.println("hot water circulating pump is on"); 
        if (hotWaterPump.SetValue(1)) CheckState();
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
        if (pump.SetValue(0)) CheckState();
        break;
      case 0x64:
        //Serial.println("internal pump is operating in the service water circuit"); 
        if (pump.SetValue(1)) CheckState();
        break;
      case 0xFE: 
        //Serial.println("internal pump is operating in the heating circuit"); 
        if (pump.SetValue(2)) CheckState();
        break;
      }
    }
  }
}

void ParseVaillantTelegram16()
{
  if (packet[5] == 0x00)
  {
    Serial.print(F("Vaillant Broadcast Service - date / time "));
    char buffer[20] = {0}; 
    snprintf(buffer, sizeof(buffer), "20%02x-%02x-%02x %02x:%02x:%02x", packet[12], packet[10], packet[9], packet[8], packet[7], packet[6]);
    Serial.println(buffer); 
    tmElements_t t;
    t.Year = 2000 + Bcd2Dec(packet[12]);
    t.Month = Bcd2Dec(packet[10]);
    t.Day = Bcd2Dec(packet[9]);
    t.Hour = Bcd2Dec(packet[8]);
    t.Minute = Bcd2Dec(packet[7]);
    t.Second = Bcd2Dec(packet[6]);
    time_t unix = makeTime(t);
    /* low level functions to convert to and from system time                     */
/*void breakTime(time_t time, tmElements_t &tm);  // break time_t into elements
time_t makeTime(tmElements_t &tm);  // convert time elements into time_t*/
    
    MyMessage msg(SENSOR_DT, V_TEXT);
    send(msg.set(buffer));
    
  }
  else if (packet[5] == 0x01)
  {
    Serial.println(F("Vaillant broadcast service"));
  }
}

void ParseVaillantTelegram()
{
  switch (packet[3])
  {
    case 0x04: ParseVaillantTelegram04(); break;
    case 0x09: ParseVaillantTelegram09(); break;
    case 0x10: ParseVaillantTelegram10(); break;
    case 0x11: ParseVaillantTelegram11(); break;
    case 0x12: ParseVaillantTelegram12(); break;
    case 0x16: ParseVaillantTelegram16(); break;
    default:
      Serial.println(F("unhandled Vaillant telegram"));
  }
}

void ReconstructTelegram()
{
  for (int i = 0; i < packetBytes; i++)
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
  Serial.print(heating.Value());
  Serial.print(" W=");
  Serial.print(water.Value());
  Serial.print("\tP=");
  Serial.print(pump.Value());
  Serial.print(" HWP=");
  Serial.print(hotWaterPump.Value());
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
  if ((now - m_Millis) > REFRESH_INTERVAL)
  {
    return true;
  };
  return false;
}

void CSensor::Touch()
{
  m_Millis = millis();
}

CFloatSensor::CFloatSensor(const byte a_ID, const byte a_Type, const char * a_Name) :
  CSensor(a_ID, a_Name),
  m_Type(a_Type)
{
  m_Value = 0;
}

bool CFloatSensor::SetValue(const float a_Value)
{
  if ((abs(a_Value - m_Value) > 1.0) || NeedsRefresh())
  {
    m_Value = a_Value;
    // Send in the new temperature
    MyMessage msg(m_ID, m_Type);
    send(msg.set(m_Value, 1));
    Touch();
    return true;
  }
  return false;
}

float CFloatSensor::Value() const
{
  return m_Value; 
}

CBitSensor::CBitSensor(const byte a_ID, const byte a_Type, const char * a_Name) :
  CSensor(a_ID, a_Name),
  m_Type(a_Type)
{
  m_Value = false;
}

bool CBitSensor::SetValue(const bool a_Value)
{
  if ((a_Value != m_Value) || NeedsRefresh())
  {
    m_Value = a_Value;
    // Send in the new temperature
    MyMessage msg(m_ID, m_Type);
    send(msg.set(m_Value ? 1 : 0));
    Touch();
    return true;
  }
  return false;
}

bool CBitSensor::Value() const
{
  return m_Value;
}

CByteSensor::CByteSensor(const byte a_ID, const byte a_Type, const char * a_Name) :
  CSensor(a_ID, a_Name),
  m_Type(a_Type)
{
  m_Value = 0;
}

bool CByteSensor::SetValue(const byte a_Value)
{
  if ((a_Value != m_Value) || NeedsRefresh())
  {
    m_Value = a_Value;
    // Send in the new temperature
    MyMessage msg(m_ID, m_Type);
    send(msg.set(m_Value));
    Touch();
    return true;
  }
  return false;
}

byte CByteSensor::Value() const
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
