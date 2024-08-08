#include "emotion.h"

void setup()
{
  Emotion_Setup();
  emotion_set_task_mode(6);
}

void loop()
{
  arrow_set_mode(1);
  delay(3000);
  arrow_set_mode(2);
  delay(3000);
  arrow_set_mode(3);
  delay(3000);
  arrow_set_mode(4);
  delay(3000);
}
