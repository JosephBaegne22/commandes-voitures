#include "Freenove_VK16K33_Lib_For_ESP32.h"
#include "Emotion.h"
#include "Array.h"

Freenove_ESP32_VK16K33 matrix = Freenove_ESP32_VK16K33();

void show_arrow(int arrow_direction)
{
  arrow_direction = constrain(arrow_direction, 0, 5);

  if (arrow_direction == 1)
  {
    for (int j = -8; j <= 8; j++)
    {
      matrix.showLedMatrix(arrow_up, 4, j);
      delay(100);
    }
  }
  if (arrow_direction == 2)
  {
    for (int j = 8; j >= -8; j--)
    {
      matrix.showLedMatrix(arrow_dowm, 4, j);
      delay(100);
    }
  }
  if (arrow_direction == 3)
  {
    for (int i = 8; i >= 0; i--)
    {
      matrix.showLedMatrix(arrow_left, i, 0);
      delay(100);
    }
  }
  if (arrow_direction == 4)
  {
    for (int i = 0; i <= 8; i++)
    {
      matrix.showLedMatrix(arrow_right, i, 0);
      delay(100);
    }
  }
}

//Turn the eyes
void eyesRotate(int delay_ms)
{
  int count = sizeof(eyeRotate1) / sizeof(eyeRotate1[0]);
  for (int i = 0; i < count; i++)
  {
    matrix.showStaticArray(eyeRotate1[i], eyeRotate2[i]);
    delay(delay_ms);
  }
}

//Wink the eyes
void eyesBlink(int delay_ms)
{
  int count = sizeof(eyeBlink) / sizeof(eyeBlink[0]);
  matrix.showStaticArray(eyeBlink[0], eyeBlink[0]);
  delay(2500);
  for (int j = 0; j < 1; j++)
  {
    for (int i = 1; i < count; i++)
    {
      matrix.showStaticArray(eyeBlink[i], eyeBlink[i]);
      delay(delay_ms);
    }
  }
}

//Wink the eyes
void eyesBlink1(int delay_ms)
{
  int count = sizeof(eyeBlink1) / sizeof(eyeSmile[0]);
  matrix.showStaticArray(eyeBlink1[0], eyeBlink1[0]);
  delay(800);
  for (int i = 0; i < count; i++)
  {
    matrix.showStaticArray(eyeBlink1[i], eyeBlink1[i]);
    delay(delay_ms);
  }
}

//Smile
void eyesSmile(int delay_ms)
{
  int count = sizeof(eyeSmile) / sizeof(eyeSmile[0]);
  for (int i = 0; i < count; i++)
  {
    matrix.showStaticArray(eyeSmile[i], eyeSmile[i]);
    delay(delay_ms);
  }
}

//Cry
void eyesCry(int delay_ms)
{
  int count = sizeof(eyeCry1) / sizeof(eyeCry1[0]);
  for (int i = 0; i < count; i++)
  {
    matrix.showStaticArray(eyeCry1[i], eyeCry2[i]);
    delay(delay_ms);
  }
}

//wheel
void wheel(int mode, int delay_ms)
{
  if (mode == 1)
  {
    int count = sizeof(wheel_left) / sizeof(wheel_left[0]);
    for (int i = 0; i < count; i++)
    {
      matrix.showStaticArray(wheel_left[i], wheel_left[i]);
      delay(delay_ms);
    }
  }
  else if (mode == 2)
  {
    int count = sizeof(wheel_right) / sizeof(wheel_right[0]);
    for (int i = 0; i < count; i++)
    {
      matrix.showStaticArray(wheel_right[i], wheel_right[i]);
      delay(delay_ms);
    }
  }
  else
    matrix.clear();
}

//Car
void carMove(int mode,int delay_ms)
{
  if (mode == 1)
  {
      for (int i = 0; i <= 8; i++)
      {
        matrix.showLedMatrix(car_left, 8-i, 0);
        delay(delay_ms);
      }
  }
  else if (mode == 2)
  {
      for (int i = 0; i <= 8; i++)
      {
        matrix.showLedMatrix(car_right, i, 0);
        delay(delay_ms);
      }
  }
  else
    matrix.clear();
}

//expressing love
void expressingLove(void)
{
  int count = sizeof(I_love_you) / sizeof(I_love_you[0]);
  for (int i = 0; i < count; i++)
  {
    matrix.showStaticArray(I_love_you[0], I_love_you[1]);
    delay(100);
  }
}

//save water
void saveWater(int delay_ms)
{
  int count = (sizeof(save_water_left) / sizeof(save_water_left[0]));
  for (int i = 0; i < count; i++)
  {
    matrix.showStaticArray(save_water_left[i], save_water_right[i]);
    delay(delay_ms);
  }
}

//Initialize
void Emotion_Setup()
{
  matrix.init(EMOTION_ADDRESS, EMOTION_SDA, EMOTION_SCL);
  xTaskCreatePinnedToCore(Emotion_Show,  "Emotion_Show",  8192,  NULL,  1,  NULL ,  0);
}

int emotion_mode = 0;
void emotion_set_task_mode(int task_mode)
{
  emotion_mode = task_mode;
}

int arrow_mode = 0;
void arrow_set_mode(int mode)
{
  arrow_mode = mode;
}

int wheel_mode = 0;
void wheel_set_mode(int mode)
{
  wheel_mode = mode;
}

int car_mode = 0;
void car_set_mode(int mode)
{
  car_mode = mode;
}

//Expression shows
void Emotion_Show(void* pvParameters)
{
  while (1)
  {
    if (emotion_mode == 1)
      eyesRotate(150);
    else if (emotion_mode == 2)
      eyesBlink(150);
    else if (emotion_mode == 3)
      eyesSmile(200);
    else if (emotion_mode == 4)
      eyesCry(200);
    else if (emotion_mode == 5)
      eyesBlink1(200);
    else if (emotion_mode == 6)
      show_arrow(arrow_mode);
    else if (emotion_mode == 7)
      wheel(wheel_mode, 100);
    else if (emotion_mode == 8)
      carMove(car_mode,100);
    else if (emotion_mode == 9)
      expressingLove();
    else if (emotion_mode == 10)
      saveWater(200);
    else if (emotion_mode == 0)
      matrix.clear();
  }
}
