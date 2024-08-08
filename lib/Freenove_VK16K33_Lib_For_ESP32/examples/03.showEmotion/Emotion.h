#ifndef _EMOTION_H
#define _EMOTION_H

#define EMOTION_ADDRESS 0x71
#define EMOTION_SDA     13
#define EMOTION_SCL     14

void Emotion_Setup();
void Emotion_Show(int mode);

#endif
