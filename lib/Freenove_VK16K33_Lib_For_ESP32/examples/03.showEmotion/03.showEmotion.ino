#include "emotion.h"

void setup()
{
  Emotion_Setup();
  Emotion_Show(1);  //Turn your eyes clockwise
  Emotion_Show(2);  //Blink your eyes
  Emotion_Show(3);  //Smile
  Emotion_Show(4);  //Cry
}

void loop()
{
  Emotion_Show(1);
}
