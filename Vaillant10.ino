void ParseVaillantTelegram10()
{
  if (packet[4] == 0x09)
  {
    Serial.print(F("Vaillant Room Controller: ")); 
    Serial.println(packet[11]);
    ProcessData1c(7, vtt);
    ProcessData1c(8, stt);
    CheckState();
  }
  else
  {
    Serial.println(F("Vaillant Room Controller - unhandled block ------------------------------------- ")); 
  }
}


