#ifndef __RGB_H
#define __RGB_H
#include <stdint.h>

#define BLING_R_PIN GPIO_PIN_5 //红色
#define BLING_G_PIN GPIO_PIN_6//绿色
#define BLING_B_PIN GPIO_PIN_4//蓝色

typedef struct
{
  uint16_t contiune_t;//闪烁持续时间
  uint16_t period;//闪烁周期
  float  percent;//闪烁占空比
  uint16_t  cnt;//闪烁计数器
  uint32_t port; //端口
  uint16_t pin;//引脚
  uint8_t endless_flag;//无尽模式
}Bling_Light;

void rgb_init(void);
void bling_working(uint16_t bling_mode);
void bling_set(Bling_Light *light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint8_t Flag);
void rgb_start_bling(void);

extern Bling_Light light_red,light_green,light_blue;

#endif
