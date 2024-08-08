#include "emotion.h"
#include <Wire.h>
void setup()
{
  Wire.beginTransmission(0x5f);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Emotion_Setup();
  emotion_set_task_mode(5);
}

void loop()
{

}
