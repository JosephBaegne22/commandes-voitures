#ifndef _EMOTION_H
#define _EMOTION_H

#define EMOTION_ADDRESS 0x71
#define EMOTION_SDA     13
#define EMOTION_SCL     14

void eyesRotate(int delay_ms);             //Turn the eyes-1
void eyesBlink(int delay_ms);              //Wink the eyes-2
void eyesBlink1(int delay_ms);             //Wink the eyes-3
void eyesSmile(int delay_ms);              //Smile-4
void eyesCry(int delay_ms);                //Cry-5
void show_arrow(int arrow_direction);      //Arrow-6
void Emotion_Show(void* pvParameters);     //Expression shows

void Emotion_Setup();
void arrow_set_mode(int mode);             //1-up;2-dowm;3-left;4-right
void emotion_set_task_mode(int task_mode);

#endif
