#include "Freenove_VK16K33_Lib_For_ESP32.h"

#define SDA 13
#define SCL 14

Freenove_ESP32_VK16K33 matrix = Freenove_ESP32_VK16K33();

//The brightness values can be set from 1 to 15, with 1 darkest and 15 brightest
#define  A  15

byte arrow[8][8] =
{
  {0, 0, 0, A, A, 0, 0, 0},
  {0, 0, A, A, A, A, 0, 0},
  {0, A, 0, A, A, 0, A, 0},
  {0, 0, 0, A, A, 0, 0, 0},
  {0, 0, 0, A, A, 0, 0, 0},
  {0, 0, 0, A, A, 0, 0, 0},
  {0, 0, 0, A, A, 0, 0, 0},
  {0, 0, 0, A, A, 0, 0, 0},
};

void setup()
{
  matrix.init(0x71, SDA, SCL);//Initialize matrix
  matrix.showLedMatrix(arrow,4,0);
  matrix.show();
}

void loop()
{
  for (int i = 0; i <= 15; i++)
  {
    matrix.setBrightness(i);
    delay(100);
  }
  for (int i = 15; i > 0; i--)
  {
    matrix.setBrightness(i);
    delay(100);
  }
}
