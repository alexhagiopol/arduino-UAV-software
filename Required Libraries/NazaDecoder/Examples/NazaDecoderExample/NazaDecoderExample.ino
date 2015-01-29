/*
  DJI Naza (v1, v1 Lite, V2) data decoder library example
  (c) Pawelsky 20140628
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "NazaDecoderLib.h"

uint32_t currTime, attiTime;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if(Serial.available())
  {
    uint8_t decodedMessage = NazaDecoder.decode(Serial.read());
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
        Serial.print("Lat: "); Serial.print(NazaDecoder.getLat(), 7);
        Serial.print(", Lon: "); Serial.print(NazaDecoder.getLon(), 7);
        Serial.print(", Alt: "); Serial.print(NazaDecoder.getGpsAlt(), 7);
        Serial.print(", Fix: "); Serial.print(NazaDecoder.getFixType());
        Serial.print(", Sat: "); Serial.println(NazaDecoder.getNumSat());
        break;
      case NAZA_MESSAGE_COMPASS:
        Serial.print("Heading: "); Serial.println(NazaDecoder.getHeadingNc(), 2);
        break;
    }
  }

  currTime = micros();

  // Display attitude at 5Hz rate so every 200000 microseconds
  if(attiTime < currTime)
  {
    attiTime = currTime + 200000;
    Serial.print("Pitch: "); Serial.print(NazaDecoder.getPitch());
    Serial.print(", Roll: "); Serial.println(NazaDecoder.getRoll());
  }
}
