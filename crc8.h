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

