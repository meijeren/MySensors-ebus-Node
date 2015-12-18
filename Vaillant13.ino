void ParseVaillantTelegram13()
{
  // 03 15 B5 13 03 06 00 00 0E # 00 01 01 9A 00
  if (packet[4] == 0x03)
  {
    Serial.print(F("Vaillant 13h - "));
    Serial.println(packet[5], HEX);
  }
  else
  {
    Serial.println(F("Vaillant 13h - unhandled block ---------------------------------------"));
  }

}

