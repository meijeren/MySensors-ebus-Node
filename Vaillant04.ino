void ParseVaillantTelegram04()
{
  if ((packet[4] == 0x01) && (packet[5] == 0x00))
  {
    Serial.print(F("Vaillant GetDataBlock - DCF77 "));
    switch (packet[7])
    {
      case 0x0:
        Serial.println("no reception");
        break;
      case 0x01: // off
        Serial.println("reception");
        break;
      case 0x02:
        Serial.println("synchronized");
        break;
      case 0x03:
        Serial.println("data valid");
        break;
    }
  }
  else
  {
    Serial.println(F("Vaillant Get Data Block - unhandled block -----------------------------------"));
  }

}
