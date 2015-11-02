#include <MyHwATMega328.h>
#include <MySensor.h>
#include <SPI.h>

/*
  Vaillant ebus sensor node


 */
#include <SoftwareSerial.h>

#define NODE_ID       0xB5
#define NODE_TEXT     "Vaillant ebus node"
#define NODE_VERSION  "0.4"

#define RECEIVE_PIN   A0
#define TRANSMIT_PIN  11
#define REFRESH_INTERVAL 3600000  // Every hour

// VTT wordt 90 als de ketel aanslaat.
// VT en NT gaan dan oplopen
#define SENSOR_VTT 1
#define SENSOR_VT 3
#define SENSOR_NT 4
#define SENSOR_WTT 5
#define SENSOR_WT 6
#define SENSOR_STT 7
#define SENSOR_ST 8
#define SENSOR_HEATING 9
#define SENSOR_WATER 10
#define SENSOR_MODEL 11
#define SENSOR_HOT_WATER_PUMP 12
#define SENSOR_PUMP 13

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

float ProcessData1C(const byte a_Value, CFloatSensor & a_Sensor);

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

CFloatSensor vtt(SENSOR_VTT, "VTT");
CFloatSensor vt(SENSOR_VT, "VT");
CFloatSensor nt(SENSOR_NT, "NT");
CFloatSensor stt(SENSOR_STT, "STT");
CFloatSensor st(SENSOR_ST, "ST");
CFloatSensor wtt(SENSOR_WTT, "WTT");
CFloatSensor wt(SENSOR_WT, "WTT");

void setup()
{
  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);

  // Initialize library and add callback for incoming messages
  gw.begin(NULL, NODE_ID, false);
  Serial.println(NODE_TEXT  " "  NODE_VERSION);
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
  gw.present(SENSOR_WATER, S_BINARY);
  gw.present(SENSOR_MODEL, S_CUSTOM);
}

float ProcessData1C(const byte a_Value, CFloatSensor & a_Sensor)
{
  if (a_Sensor.SetValue(a_Value / 2.0))
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

//////////////////////////////////////////////////////////////////////////
//
// CRC-Berechnung  aus http://www.mikrocontroller.net/topic/75698
//
//////////////////////////////////////////////////////////////////////////
 
#ifdef USE_CRC_TAB
const byte CRC_Tab8Value[256] '' {
   (UCHAR) 0x00, (UCHAR) 0x9B, (UCHAR) 0xAD, (UCHAR) 0x36, (UCHAR) 0xC1, (UCHAR) 0x5A, (UCHAR) 0x6C, (UCHAR) 0xF7,
   (UCHAR) 0x19, (UCHAR) 0x82, (UCHAR) 0xB4, (UCHAR) 0x2F, (UCHAR) 0xD8, (UCHAR) 0x43, (UCHAR) 0x75, (UCHAR) 0xEE,
   (UCHAR) 0x32, (UCHAR) 0xA9, (UCHAR) 0x9F, (UCHAR) 0x04, (UCHAR) 0xF3, (UCHAR) 0x68, (UCHAR) 0x5E, (UCHAR) 0xC5,
   (UCHAR) 0x2B, (UCHAR) 0xB0, (UCHAR) 0x86, (UCHAR) 0x1D, (UCHAR) 0xEA, (UCHAR) 0x71, (UCHAR) 0x47, (UCHAR) 0xDC,
   (UCHAR) 0x64, (UCHAR) 0xFF, (UCHAR) 0xC9, (UCHAR) 0x52, (UCHAR) 0xA5, (UCHAR) 0x3E, (UCHAR) 0x08, (UCHAR) 0x93,
   (UCHAR) 0x7D, (UCHAR) 0xE6, (UCHAR) 0xD0, (UCHAR) 0x4B, (UCHAR) 0xBC, (UCHAR) 0x27, (UCHAR) 0x11, (UCHAR) 0x8A,
   (UCHAR) 0x56, (UCHAR) 0xCD, (UCHAR) 0xFB, (UCHAR) 0x60, (UCHAR) 0x97, (UCHAR) 0x0C, (UCHAR) 0x3A, (UCHAR) 0xA1,
   (UCHAR) 0x4F, (UCHAR) 0xD4, (UCHAR) 0xE2, (UCHAR) 0x79, (UCHAR) 0x8E, (UCHAR) 0x15, (UCHAR) 0x23, (UCHAR) 0xB8,
   (UCHAR) 0xC8, (UCHAR) 0x53, (UCHAR) 0x65, (UCHAR) 0xFE, (UCHAR) 0x09, (UCHAR) 0x92, (UCHAR) 0xA4, (UCHAR) 0x3F,
   (UCHAR) 0xD1, (UCHAR) 0x4A, (UCHAR) 0x7C, (UCHAR) 0xE7, (UCHAR) 0x10, (UCHAR) 0x8B, (UCHAR) 0xBD, (UCHAR) 0x26,
   (UCHAR) 0xFA, (UCHAR) 0x61, (UCHAR) 0x57, (UCHAR) 0xCC, (UCHAR) 0x3B, (UCHAR) 0xA0, (UCHAR) 0x96, (UCHAR) 0x0D,
   (UCHAR) 0xE3, (UCHAR) 0x78, (UCHAR) 0x4E, (UCHAR) 0xD5, (UCHAR) 0x22, (UCHAR) 0xB9, (UCHAR) 0x8F, (UCHAR) 0x14,
   (UCHAR) 0xAC, (UCHAR) 0x37, (UCHAR) 0x01, (UCHAR) 0x9A, (UCHAR) 0x6D, (UCHAR) 0xF6, (UCHAR) 0xC0, (UCHAR) 0x5B,
   (UCHAR) 0xB5, (UCHAR) 0x2E, (UCHAR) 0x18, (UCHAR) 0x83, (UCHAR) 0x74, (UCHAR) 0xEF, (UCHAR) 0xD9, (UCHAR) 0x42,
   (UCHAR) 0x9E, (UCHAR) 0x05, (UCHAR) 0x33, (UCHAR) 0xA8, (UCHAR) 0x5F, (UCHAR) 0xC4, (UCHAR) 0xF2, (UCHAR) 0x69,
   (UCHAR) 0x87, (UCHAR) 0x1C, (UCHAR) 0x2A, (UCHAR) 0xB1, (UCHAR) 0x46, (UCHAR) 0xDD, (UCHAR) 0xEB, (UCHAR) 0x70,
   (UCHAR) 0x0B, (UCHAR) 0x90, (UCHAR) 0xA6, (UCHAR) 0x3D, (UCHAR) 0xCA, (UCHAR) 0x51, (UCHAR) 0x67, (UCHAR) 0xFC,
   (UCHAR) 0x12, (UCHAR) 0x89, (UCHAR) 0xBF, (UCHAR) 0x24, (UCHAR) 0xD3, (UCHAR) 0x48, (UCHAR) 0x7E, (UCHAR) 0xE5,
   (UCHAR) 0x39, (UCHAR) 0xA2, (UCHAR) 0x94, (UCHAR) 0x0F, (UCHAR) 0xF8, (UCHAR) 0x63, (UCHAR) 0x55, (UCHAR) 0xCE,
   (UCHAR) 0x20, (UCHAR) 0xBB, (UCHAR) 0x8D, (UCHAR) 0x16, (UCHAR) 0xE1, (UCHAR) 0x7A, (UCHAR) 0x4C, (UCHAR) 0xD7,
   (UCHAR) 0x6F, (UCHAR) 0xF4, (UCHAR) 0xC2, (UCHAR) 0x59, (UCHAR) 0xAE, (UCHAR) 0x35, (UCHAR) 0x03, (UCHAR) 0x98,
   (UCHAR) 0x76, (UCHAR) 0xED, (UCHAR) 0xDB, (UCHAR) 0x40, (UCHAR) 0xB7, (UCHAR) 0x2C, (UCHAR) 0x1A, (UCHAR) 0x81,
   (UCHAR) 0x5D, (UCHAR) 0xC6, (UCHAR) 0xF0, (UCHAR) 0x6B, (UCHAR) 0x9C, (UCHAR) 0x07, (UCHAR) 0x31, (UCHAR) 0xAA,
   (UCHAR) 0x44, (UCHAR) 0xDF, (UCHAR) 0xE9, (UCHAR) 0x72, (UCHAR) 0x85, (UCHAR) 0x1E, (UCHAR) 0x28, (UCHAR) 0xB3,
   (UCHAR) 0xC3, (UCHAR) 0x58, (UCHAR) 0x6E, (UCHAR) 0xF5, (UCHAR) 0x02, (UCHAR) 0x99, (UCHAR) 0xAF, (UCHAR) 0x34,
   (UCHAR) 0xDA, (UCHAR) 0x41, (UCHAR) 0x77, (UCHAR) 0xEC, (UCHAR) 0x1B, (UCHAR) 0x80, (UCHAR) 0xB6, (UCHAR) 0x2D,
   (UCHAR) 0xF1, (UCHAR) 0x6A, (UCHAR) 0x5C, (UCHAR) 0xC7, (UCHAR) 0x30, (UCHAR) 0xAB, (UCHAR) 0x9D, (UCHAR) 0x06,
   (UCHAR) 0xE8, (UCHAR) 0x73, (UCHAR) 0x45, (UCHAR) 0xDE, (UCHAR) 0x29, (UCHAR) 0xB2, (UCHAR) 0x84, (UCHAR) 0x1F,
   (UCHAR) 0xA7, (UCHAR) 0x3C, (UCHAR) 0x0A, (UCHAR) 0x91, (UCHAR) 0x66, (UCHAR) 0xFD, (UCHAR) 0xCB, (UCHAR) 0x50,
   (UCHAR) 0xBE, (UCHAR) 0x25, (UCHAR) 0x13, (UCHAR) 0x88, (UCHAR) 0x7F, (UCHAR) 0xE4, (UCHAR) 0xD2, (UCHAR) 0x49,
   (UCHAR) 0x95, (UCHAR) 0x0E, (UCHAR) 0x38, (UCHAR) 0xA3, (UCHAR) 0x54, (UCHAR) 0xCF, (UCHAR) 0xF9, (UCHAR) 0x62,
   (UCHAR) 0x8C, (UCHAR) 0x17, (UCHAR) 0x21, (UCHAR) 0xBA, (UCHAR) 0x4D, (UCHAR) 0xD6, (UCHAR) 0xE0, (UCHAR) 0x7B
};
 
 
 
/********************************************************************************************************/
/** Function for CRC-calculation with tab operations  */
/********************************************************************************************************/
byte crc8(byte data, byte crc_init)
{
   byte crc;
 
   crc = (byte) (CRC_Tab8Value[crc_init] ^ data);
   return (crc);
}
 
#else
 
 
/********************************************************************************************************/
/** slower, but less memory                        */
/********************************************************************************************************/
unsigned char crc8(unsigned char data, unsigned char crc_init)
{
   unsigned char crc;
   unsigned char polynom;
   int i;
 
   crc = crc_init;
   for (i = 0; i < 8; i++)
   {
      if (crc & 0x80)
      {
         polynom = (unsigned char) 0x9B;
      }
      else
      {
         polynom = (unsigned char) 0;
      }
      crc = (unsigned char)((crc & ~0x80) << 1);
      if (data & 0x80)
      {
         crc = (unsigned char)(crc | 1) ;
      }
      crc = (unsigned char)(crc ^ polynom);
      data = (unsigned char)(data << 1);
   }
   return (crc);
}
#endif
 
 
byte CalculateCRC(byte* & data, int len )
{
   byte crc = 0;
   for (int i = 0 ; i < len ; ++i, ++data )
   {
      crc = crc8(*data, crc );
   }
   return crc;
}

void ReconstructTelegram();

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
          Serial.println("Not enough bytes!");
          packetBytes = 0;
          return;
        }
        byte crc = packet[5 + length];
        byte * data = &packet[0];
        if (CalculateCRC(data, 5 + length) != crc)
        {
          Serial.println("CRC error");
          packetBytes = 0;
          return;
        }
        ReconstructTelegram();
        if (packet[2] == 0xB5)
        {
          // Vaillant
          if (packet[3] == 0x04)
          {
            if (packet[5] == 0x01)
            {
              if (packetBytes >= 11)
              {
                  Serial.print("Vaillant Get Operating Mode: TV"); 
                  Serial.print(packet[9], HEX);
                  Serial.print(" op.mode=");
                  Serial.println(packet[10], HEX);
              }
            }
            else if (packet[5] == 0x00)
            {
              Serial.println("Vaillant Get Operational Data - data/time");
            }
            else
            {
              Serial.println("Vaillant Get Operational Data - unhandled block");
            }
          }
          else if (packet[3] == 0x10)
          {
            if (packetBytes >= 16)
            {
              Serial.print("Vaillant Room Controller: "); 
              ProcessData1C(packet[7], vtt);
              ProcessData1C(packet[8], stt);
              Serial.println();
            }
          }
          else if (packet[3] == 0x11)
          {
            if (packet[5] == 0x01)
            {
              Serial.println("Vaillant Burner Control Unit - block 1"); 
              ProcessData1C(packet[9], vt);
              ProcessData1C(packet[10], nt);
              ProcessData1C(packet[12], wt);
              ProcessData1C(packet[13], st);
              ProcessDataBit(packet[14], 0, heating, SENSOR_HEATING, "H");
              ProcessDataBit(packet[14], 1, water, SENSOR_WATER, "W");
            }
            if (packet[5] == 0x02)
            {
              Serial.println("Vaillant Burner Control Unit - block 2"); 
              ProcessData1C(packet[13], wtt);
            }
          }
          else if (packet[3] == 0x12)
          {
            Serial.println("Vaillant pump");
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
              Serial.print("Vaillant Broadcast Service - date / time "); 
              Serial.print(packet[12], HEX);
              Serial.print(":");
              Serial.print(packet[10], HEX);
              Serial.print(":");
              Serial.print(packet[9], HEX);
              Serial.print(" ");
              Serial.print(packet[11], HEX);
              Serial.print(" ");
              Serial.print(packet[8], HEX);
              Serial.print("-");
              Serial.print(packet[7], HEX);
              Serial.print("-");
              Serial.print(packet[6], HEX);
              Serial.println(); 
            }
            else if (packet[6] == 0x01)
            {
              Serial.println("Vaillant broadcast servce");
            }
          }
          else
          {
            Serial.println("unhandled Vaillant telegram");
          }
        }
        else if (packet[2] == 0x07)
        {
          if (packetBytes >= 20)
          {
            Serial.print("Identification: ");
            Serial.print(char(packet[9]));
            Serial.print(char(packet[10]));
            Serial.print(char(packet[11]));
            Serial.print(char(packet[12]));
            Serial.print(char(packet[13]));
            Serial.print("\tSW-Version: ");
            Serial.print(packet[14]);
            Serial.print(".");
            Serial.print(packet[15]);
            Serial.print(" HW-Version: ");
            Serial.print(packet[16]);
            Serial.print(".");
            Serial.print(packet[17]);
            Serial.println();
          }
        }
        else 
        {
          Serial.println("Unhandled telegram");
        }
        packetBytes = 0;
      }
    }
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
