void ParseVaillantTelegram05()
{
  if ((packet[5] == 0x02) && (packet[6] == 0x02))
  {
    Serial.print(F("Vaillant SetOperationMode: "));
    switch (packet[7])
    {
      case 0x01: // on (heating);
        Serial.println("on (heating)");
        break;
      case 0x02: // off
        Serial.println("off");
        break;
      case 0x03: // auto
        Serial.println("auto");
        break;
      case 0x04: // eco
        Serial.println("eco");
        break;
      case 0x05:
        Serial.println("night set back");
        break;
    }
  }
  else
  {
    Serial.println(F("Vaillant SetOperationalData - unhandled block -------------------------------"));
  }
}
