void ParseVaillantTelegram16()
{
  if ((packet[4] == 0x08) && (packet[5] == 0x00))
  {
    Serial.print(F("Vaillant BroadcastService - date/time "));
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
  else if ((packet[4] == 0x03) && (packet[5] == 0x01))
  {
    Serial.print(F("Vaillant BroadcastService - Outside temperature: "));
    float value;
    byte a_Offset = 6;
    if ((packet[a_Offset+1] & 0x80) == 0x80)
    {
      value = -packet[a_Offset+1] + ((packet[a_Offset] + 1.0) / 256);
    }
    else
    {
      value = packet[a_Offset+1] + ((packet[a_Offset] + 1.0) / 256);
    }
    Serial.println(value);
  }
  else
  {
    Serial.println(F("Vaillant BroadcastService - unhandled block ------------------------------------"));
  }
}

