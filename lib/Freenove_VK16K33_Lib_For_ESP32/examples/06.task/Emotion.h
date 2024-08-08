#ifndef _EMOTION_H
#define _EMOTION_H

#define EMOTION_ADDRESS 0x71
#define EMOTION_SDA     13
#define EMOTION_SCL     14

void eyesRotate(int delay_ms);             //Turn the eyes
void eyesBlink(int delay_ms);              //Wink the eyes
void eyesSmile(int delay_ms);              //Smile
void eyesCry(int delay_ms);                //Cry
void eyesBlink1(int delay_ms);             //Wink the eyes
void show_arrow(int arrow_direction);      //Arrow
void wheel(int mode,int delay_ms);         //wheel
void carMove(int mode,int delay_ms);       //car
void expressingLove(void);                 //expressing love
void saveWater(int delay_ms);              //save water

void Emotion_Show(void* pvParameters);     //Expression shows
void Emotion_Setup();
void arrow_set_mode(int mode);             //arrow: 1-up;2-dowm;3-left;4-right
void wheel_set_mode(int mode);             //wheel: 1-turn left;2-turn right
void car_set_mode(int mode);               //car：1-left;2-right

void emotion_set_task_mode(int task_mode); //Set different led matrix modes

#endif
