//sensor重命名
#include <IRremote.h>
#include <Arduino.h>
#include "U8glib.h"
//#include "utility/u8g.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST); // Fast I2C / TWI 

//不要使用自定义format函数，函数内字符串拼接有bug，导致死机
/*
 * 红外传感器数目，2*2
 * 传感器位置
 *  S1_1   S1_2
 *  S2_1   S2_2 
 */
const int SENSOR_COUNT = 4;
const int ACTION_COUNT=6;
//const int LED_PING_FOR_ACTION[6]={0,9,12,11,10,8};
//const String ACTION_NAMES[6]={"Invalid","Left","Right","Up","Down","Start/Stop"};
const  String ACTION_NAMES[6]={"未知","左","右","上","下","开关"};
//上次打印debug时间，用于控制debug输出内容的时间间隔
unsigned long lastDebugTime=0;
//红外遥控相关常亮
//int RECV_PIN = 11;
//IRrecv irrecv(RECV_PIN);
IRsend irsend;
//红外线代码
const long IR_STOP_RESUME=0xFF02FD;
const long IR_NEXT=0xFFF00F;
const long IR_PREVIOUS=0xFFA05F;
const long IR_VOL_UP=0xFF40BF;
const long IR_VOL_DOWN=0xFFE01F;
const long IR_CODES[6]={0,IR_PREVIOUS,IR_NEXT,IR_VOL_UP,IR_VOL_DOWN,IR_STOP_RESUME};

const int buttonPin = 10;     // the number of the pushbutton pin

int buttonState = LOW;         // variable for reading the pushbutton status
int functionType=0;//遥控功能，0：收音机，1：电视机

//排序用时间数组
struct Sensor {
  int sensorId;//传感器对应下标
  float curV;//当前电压
  float avgV;//平均电压
  float peakV;//最大电压
  unsigned long peakTime;//传感器最大波峰时间(ms)
  unsigned long entryTime;//传感器进入波峰时间(ms)
  unsigned long exitTime;//传感器结束波峰时间(ms)
};

Sensor sensors[SENSOR_COUNT];
//https://www.arduino.cn/thread-31016-1-1.html
//收音机
const   uint8_t bitmap_radio1 []   U8G_PROGMEM  ={  
0x08,0x40,0x08,0x40,0x48,0x40,0x48,0x80,0x48,0xFE,0x49,0x08,0x4A,0x88,0x48,0x88,
0x48,0x88,0x58,0x50,0x68,0x50,0x48,0x20,0x08,0x50,0x08,0x88,0x09,0x04,0x0A,0x02//收0
};
const   uint8_t bitmap_radio2 []   U8G_PROGMEM  ={  
0x02,0x00,0x01,0x00,0x3F,0xF8,0x00,0x00,0x08,0x20,0x04,0x40,0xFF,0xFE,0x00,0x00,
0x1F,0xF0,0x10,0x10,0x10,0x10,0x1F,0xF0,0x10,0x10,0x10,0x10,0x1F,0xF0,0x10,0x10//音1
};
const   uint8_t bitmap_radio3 []   U8G_PROGMEM  ={  
0x10,0x00,0x11,0xF0,0x11,0x10,0x11,0x10,0xFD,0x10,0x11,0x10,0x31,0x10,0x39,0x10,
0x55,0x10,0x55,0x10,0x91,0x10,0x11,0x12,0x11,0x12,0x12,0x12,0x12,0x0E,0x14,0x00//机2
};

//电视机
const   uint8_t bitmap_tv1 []   U8G_PROGMEM  ={  
0x01,0x00,0x01,0x00,0x01,0x00,0x3F,0xF8,0x21,0x08,0x21,0x08,0x21,0x08,0x3F,0xF8,
0x21,0x08,0x21,0x08,0x21,0x08,0x3F,0xF8,0x21,0x0A,0x01,0x02,0x01,0x02,0x00,0xFE//电0
};
const   uint8_t bitmap_tv2 []   U8G_PROGMEM  ={  
0x20,0x00,0x11,0xFC,0x11,0x04,0xF9,0x04,0x09,0x24,0x11,0x24,0x11,0x24,0x39,0x24,
0x55,0x24,0x95,0x54,0x10,0x50,0x10,0x90,0x10,0x90,0x11,0x12,0x12,0x12,0x14,0x0E//视1
};
const   uint8_t bitmap_tv3 []   U8G_PROGMEM  ={  
0x10,0x00,0x11,0xF0,0x11,0x10,0x11,0x10,0xFD,0x10,0x11,0x10,0x31,0x10,0x39,0x10,
0x55,0x10,0x55,0x10,0x91,0x10,0x11,0x12,0x11,0x12,0x12,0x12,0x12,0x0E,0x14,0x00//机2
};


//?(0)  
const   uint8_t bitmap_unknown []   U8G_PROGMEM  ={  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x0F,0x80,0x00,0x00,0x00,0x00,0x3F,0xE0,0x00,0x00,0x00,
0x00,0x60,0xF8,0x00,0x00,0x00,0x00,0xC0,0x7C,0x00,0x00,0x00,0x01,0x80,0x3C,0x00,
0x00,0x00,0x01,0x00,0x3E,0x00,0x00,0x00,0x03,0x80,0x1E,0x00,0x00,0x00,0x03,0xC0,
0x1E,0x00,0x00,0x00,0x03,0xC0,0x3C,0x00,0x00,0x00,0x01,0xC0,0x3C,0x00,0x00,0x00,
0x00,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0xF0,0x00,
0x00,0x00,0x00,0x01,0xE0,0x00,0x00,0x00,0x00,0x01,0xC0,0x00,0x00,0x00,0x00,0x03,
0x80,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,
0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,
0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,
0x00,0x0F,0x80,0x00,0x00,0x00,0x00,0x1F,0xC0,0x00,0x00,0x00,0x00,0x1F,0xC0,0x00,
0x00,0x00,0x00,0x1F,0xC0,0x00,0x00,0x00,0x00,0x0F,0x80,0x00,0x00,0x00,0x00,0x07,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 //"？",0
};

//左(1)  
const   uint8_t bitmap_left []   U8G_PROGMEM  ={  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,
0x00,0x00,0x00,0x00,0x1C,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,
0x3C,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x00,
0x00,0x00,0x38,0x00,0x01,0x00,0x00,0x00,0x38,0x00,0x03,0x80,0x00,0x00,0x38,0x00,
0x07,0xC0,0x07,0xFF,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,
0x70,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x60,0x00,0x00,0x00,
0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x00,0xC0,0x00,
0x00,0x00,0x00,0x01,0xC0,0x00,0x00,0x00,0x00,0x01,0xC0,0x00,0x00,0x00,0x00,0x01,
0x80,0x00,0x00,0x00,0x00,0x03,0x80,0x00,0x00,0x00,0x00,0x03,0x80,0x00,0x18,0x00,
0x00,0x07,0x00,0x00,0x3C,0x00,0x00,0x07,0xFF,0xFF,0xFE,0x00,0x00,0x06,0x00,0x70,
0x00,0x00,0x00,0x0E,0x00,0x70,0x00,0x00,0x00,0x0C,0x00,0x70,0x00,0x00,0x00,0x18,
0x00,0x70,0x00,0x00,0x00,0x38,0x00,0x70,0x00,0x00,0x00,0x30,0x00,0x70,0x00,0x00,
0x00,0x60,0x00,0x70,0x00,0x00,0x00,0x60,0x00,0x70,0x00,0x00,0x00,0xC0,0x00,0x70,
0x00,0x00,0x01,0x80,0x00,0x70,0x00,0x00,0x03,0x00,0x00,0x70,0x00,0x00,0x03,0x00,
0x00,0x70,0x00,0x00,0x06,0x00,0x00,0x70,0x00,0x00,0x0C,0x00,0x00,0x70,0x00,0x80,
0x10,0x00,0x00,0x70,0x01,0xC0,0x20,0x00,0x00,0x70,0x03,0xE0,0x00,0x3F,0xFF,0xFF,
0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
}; 

//右(2)  
const   uint8_t bitmap_right []   U8G_PROGMEM  ={  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x07,0x80,0x00,0x00,0x00,0x00,
0x07,0x80,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,
0x00,0x00,0x0E,0x00,0x00,0x00,0x00,0x00,0x0E,0x00,0x00,0x40,0x00,0x00,0x0E,0x00,
0x00,0xE0,0x00,0x00,0x0C,0x00,0x01,0xF0,0x1F,0xFF,0xFF,0xFF,0xFF,0xF8,0x08,0x00,
0x1C,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x00,
0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x60,0x00,
0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x01,
0xC0,0x00,0x00,0x00,0x00,0x01,0x80,0x00,0x0C,0x00,0x00,0x03,0xC0,0x00,0x0E,0x00,
0x00,0x07,0xFF,0xFF,0xFF,0x00,0x00,0x06,0xC0,0x00,0x0E,0x00,0x00,0x0E,0xC0,0x00,
0x0C,0x00,0x00,0x1C,0xC0,0x00,0x0C,0x00,0x00,0x38,0xC0,0x00,0x0C,0x00,0x00,0x30,
0xC0,0x00,0x0C,0x00,0x00,0x60,0xC0,0x00,0x0C,0x00,0x00,0xC0,0xC0,0x00,0x0C,0x00,
0x01,0x80,0xC0,0x00,0x0C,0x00,0x03,0x00,0xC0,0x00,0x0C,0x00,0x06,0x00,0xC0,0x00,
0x0C,0x00,0x0C,0x00,0xC0,0x00,0x0C,0x00,0x10,0x00,0xC0,0x00,0x0C,0x00,0x00,0x00,
0xC0,0x00,0x0C,0x00,0x00,0x00,0xC0,0x00,0x0C,0x00,0x00,0x00,0xFF,0xFF,0xFE,0x00,
0x00,0x00,0xC0,0x00,0x0E,0x00,0x00,0x00,0xC0,0x00,0x0E,0x00,0x00,0x00,0xC0,0x00,
0x0E,0x00,0x00,0x00,0x80,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
//上(3)  
const   uint8_t bitmap_up []   U8G_PROGMEM  ={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x00,
0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x07,0x80,0x00,0x00,0x00,0x00,
0x07,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,
0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,
0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,
0x00,0x00,0x06,0x00,0x04,0x00,0x00,0x00,0x06,0x00,0x0E,0x00,0x00,0x00,0x06,0x00,
0x1F,0x00,0x00,0x00,0x07,0xFF,0xFF,0x80,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,
0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,
0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,
0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,
0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x40,0x00,0x00,0x06,0x00,0x00,0xE0,
0x00,0x00,0x06,0x00,0x01,0xF0,0x1F,0xFF,0xFF,0xFF,0xFF,0xF8,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
//下(4)  
const   uint8_t bitmap_down []   U8G_PROGMEM  ={  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,
0x00,0x00,0x01,0xC0,0x00,0x00,0x00,0x00,0x03,0xE0,0x0F,0xFF,0xFF,0xFF,0xFF,0xF0,
0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,
0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,
0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,
0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0xE0,
0x00,0x00,0x00,0x00,0x03,0x3C,0x00,0x00,0x00,0x00,0x03,0x1F,0x00,0x00,0x00,0x00,
0x03,0x07,0xE0,0x00,0x00,0x00,0x03,0x03,0xF8,0x00,0x00,0x00,0x03,0x00,0xFC,0x00,
0x00,0x00,0x03,0x00,0x7C,0x00,0x00,0x00,0x03,0x00,0x3E,0x00,0x00,0x00,0x03,0x00,
0x1E,0x00,0x00,0x00,0x03,0x00,0x0E,0x00,0x00,0x00,0x03,0x00,0x06,0x00,0x00,0x00,
0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,
0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,
0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,
0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,
0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x07,0x00,
0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x07,0x80,0x00,0x00,0x00,0x00,
0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

//长
const   uint8_t bitmap_kai []   U8G_PROGMEM  ={  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,
0x00,0x00,0x00,0x0F,0x00,0x00,0x60,0x00,0x00,0x0F,0x80,0x00,0xF0,0x00,0x00,0x07,
0x00,0x00,0xF8,0x00,0x00,0x06,0x00,0x01,0xF0,0x00,0x00,0x06,0x00,0x03,0xC0,0x00,
0x00,0x06,0x00,0x07,0x80,0x00,0x00,0x06,0x00,0x0F,0x00,0x00,0x00,0x06,0x00,0x1E,
0x00,0x00,0x00,0x06,0x00,0x38,0x00,0x00,0x00,0x06,0x00,0x70,0x00,0x00,0x00,0x06,
0x00,0xE0,0x00,0x00,0x00,0x06,0x01,0xC0,0x00,0x00,0x00,0x06,0x03,0x00,0x00,0x00,
0x00,0x06,0x0E,0x00,0x00,0x00,0x00,0x06,0x18,0x00,0x00,0x00,0x00,0x06,0x30,0x00,
0x00,0x00,0x00,0x06,0x40,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x01,0xC0,0x00,0x06,
0x00,0x00,0x03,0xE0,0x1F,0xFF,0xFF,0xFF,0xFF,0xF0,0x00,0x06,0x03,0x00,0x00,0x00,
0x00,0x06,0x01,0x00,0x00,0x00,0x00,0x06,0x01,0x80,0x00,0x00,0x00,0x06,0x00,0x80,
0x00,0x00,0x00,0x06,0x00,0xC0,0x00,0x00,0x00,0x06,0x00,0x60,0x00,0x00,0x00,0x06,
0x00,0x70,0x00,0x00,0x00,0x06,0x00,0x38,0x00,0x00,0x00,0x06,0x00,0x1C,0x00,0x00,
0x00,0x06,0x00,0x1E,0x00,0x00,0x00,0x06,0x00,0x0F,0x00,0x00,0x00,0x06,0x00,0x07,
0x80,0x00,0x00,0x06,0x00,0x03,0xE0,0x00,0x00,0x06,0x00,0x61,0xF8,0x00,0x00,0x06,
0x01,0xC0,0x7F,0x00,0x00,0x06,0x0F,0x00,0x3F,0xE0,0x00,0x06,0x3C,0x00,0x1F,0xFC,
0x00,0x0E,0xF0,0x00,0x07,0xE0,0x00,0x0F,0xE0,0x00,0x01,0xC0,0x00,0x0F,0x80,0x00,
0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

//按
const  uint8_t bitmap_guan []   U8G_PROGMEM  ={  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x18,
0x00,0x00,0x00,0x38,0x00,0x0C,0x00,0x00,0x00,0x3C,0x00,0x06,0x00,0x00,0x00,0x38,
0x00,0x07,0x00,0x00,0x00,0x38,0x00,0x03,0x80,0x00,0x00,0x38,0x00,0x03,0x80,0x00,
0x00,0x38,0x00,0x03,0x80,0x00,0x00,0x38,0x02,0x01,0x00,0x20,0x00,0x38,0x02,0x00,
0x00,0x70,0x00,0x39,0x07,0xFF,0xFF,0xF8,0x00,0x3B,0x86,0x00,0x00,0xF0,0x1F,0xFF,
0xEE,0x00,0x00,0xE0,0x00,0x38,0x1E,0x08,0x00,0xC0,0x00,0x38,0x1C,0x0E,0x01,0x80,
0x00,0x38,0x18,0x1E,0x01,0x00,0x00,0x38,0x00,0x1C,0x00,0x00,0x00,0x38,0x00,0x1C,
0x00,0x00,0x00,0x38,0x40,0x38,0x00,0x00,0x00,0x38,0x80,0x38,0x00,0x00,0x00,0x3B,
0x00,0x70,0x00,0x60,0x00,0x3E,0x00,0x70,0x00,0xF0,0x00,0x38,0x7F,0xFF,0xFF,0xF8,
0x00,0xF8,0x00,0xE0,0x0E,0x00,0x03,0xF8,0x00,0xC0,0x0E,0x00,0x0F,0xB8,0x01,0xC0,
0x0E,0x00,0x1F,0x38,0x01,0xC0,0x1C,0x00,0x1E,0x38,0x03,0x80,0x1C,0x00,0x0C,0x38,
0x03,0x80,0x18,0x00,0x00,0x38,0x07,0x00,0x38,0x00,0x00,0x38,0x07,0x00,0x38,0x00,
0x00,0x38,0x0F,0x00,0x70,0x00,0x00,0x38,0x01,0xF0,0x70,0x00,0x00,0x38,0x00,0x3E,
0xE0,0x00,0x00,0x38,0x00,0x07,0xE0,0x00,0x00,0x38,0x00,0x01,0xF8,0x00,0x00,0x38,
0x00,0x03,0xBE,0x00,0x00,0x38,0x00,0x07,0x1F,0x00,0x00,0x38,0x00,0x1E,0x07,0xC0,
0x00,0x38,0x00,0x3C,0x03,0xE0,0x0E,0x38,0x00,0xF0,0x01,0xE0,0x03,0xF8,0x03,0xC0,
0x00,0xE0,0x00,0xF0,0x1E,0x00,0x00,0x60,0x00,0x70,0xF0,0x00,0x00,0x00,0x00,0x40,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};


void initVArray() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].sensorId = i;
    sensors[i].curV = 0;
    sensors[i].avgV = 0;
    sensors[i].peakV = 0;
    sensors[i].peakTime = 0;
    sensors[i].entryTime = 0;
    sensors[i].exitTime = 0;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);

  Serial.println("Let's go!");

    // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  initVArray();
  draw(0);
}

//判断当前电压是否高出平均值
int isAboveAvgV(Sensor s){
  //判断波峰的最小值必须是平均值的1.5倍，并且要比平均值大50以上，避免误差
 if( s.curV > s.avgV*1.5f&&s.curV>s.avgV+50){
    return 1;
 }
 else{
  return 0;
 }
}
//判断当前电压是否回到平均值
int isBacktoAvgV(Sensor s){
 if( s.curV < s.avgV*1.2f){
    return 1;
 }
 else{
  return 0;
 }
}

//读取所有传感器的电压，放在最后一个位置
void readSensorData() {
  unsigned long t = millis();
  //  shiftSensorData();
  //传感器输入pin，数组下标按照从左到右、从上到下方式排列
  const int INPUT_PIN[SENSOR_COUNT] = {A0, A1, A2, A3};
  
  //计算最近10秒平均电压所需的取样次数，该值只影响取平均值的平滑度
  const float AVG_V_SAMPLE_COUNT = 10.0f * 1000 ;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    Sensor & s=sensors[i];
    //清理掉3秒之前的数据，一个手势动作的最长时间不应该超过3秒
    if(s.peakV>0&&t-s.peakTime>3000){
       s.peakV = 0;
       s.entryTime = 0;
       s.exitTime = 0;
       s.peakTime = 0;
    }
    //读取数据
    s.curV = analogRead(INPUT_PIN[i]);
    if(s.avgV<0.1){
      s.avgV=s.curV;
    }
    else{
      s.avgV = (s.avgV * (AVG_V_SAMPLE_COUNT - 1) + s.curV) / AVG_V_SAMPLE_COUNT;
    }
    //电压超过平均值的y%，才设置波峰
    if(isAboveAvgV(sensors[i])){
      if(s.entryTime==0){
        s.entryTime=t;
      }
      if (s.curV > s.peakV   ) {
        s.peakV = s.curV;
        s.peakTime = t;
      }
    }
    else if( isBacktoAvgV(sensors[i])){
      if(s.exitTime==0&&s.entryTime>0){
        s.exitTime=t;
      }
    }
  }
}
int debugCount=0;
void printSimpleSensorData(Sensor sensors[]){
  if(debugCount>3000){
    return;//仅打印前面3千个记录
  }
  debugCount++;
  if(debugCount==0){
      Serial.println("S_1_1, S_1_2, S_2_1, S_2_2");
  }
  debugCount++;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print((int)sensors[i].curV);
    Serial.print("\t");
  }
  Serial.println();
}

void printSensorData(int printInfo,String info,Sensor sensors[]){
  if(printInfo!=0){
      Serial.println(String("***********  ")+info+"  ***********");
  }
  const String SENSOR_NAMES[SENSOR_COUNT] = {"S_1_1", "S_1_2", "S_2_1", "S_2_2"};
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Sensor &s=sensors[i];
    int sensorId=sensors[i].sensorId;
    Serial.println(SENSOR_NAMES[sensorId]+"  sensorId:"+sensorId+"  cur:"+(int)s.curV+"  peakV:"+(int)s.peakV+"  avgV:"+(int)s.avgV
    +"  entryTime:" +(s.entryTime/1000)+(".")+(s.entryTime % 1000)
    +"  peakTime:" +(s.peakTime/1000)+(".")+(s.peakTime % 1000)
    +"  exitTime:" +(s.exitTime/1000)+(".")+(s.exitTime % 1000)
    );
  }
}
/**
   判断是否有完成的手势，判断条件：
   1：所有Sensor的当前电压不超过平均值的x%
   2: 至少有一个Sensor有1个电压超过平均值的y%
*/
int hasCompletedAction() {
  //判断所有sensor当前电压不满足peak条件
  int hasPeak=0;
  for (int s = 0; s < SENSOR_COUNT; s++) {
    if (!isBacktoAvgV(sensors[s])==1) {
      hasPeak=1;
      break;
    }
  }
  if(hasPeak==1){
    return 0;
  }
  
  //至少有一个Sensor有peak值
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensors[i].peakV >0.1) {
      return 1;
    }
  }
  //没有找到超出平均值的peakV
  return 0;
}

void draw(int actionId) {
    u8g.firstPage();
    do  {
      u8g.setColorIndex(1);
      if(functionType==0){
        u8g.drawBitmapP ( 8 , 0 , 2 , 16 , bitmap_radio1); //参数：x,y,横向字节数(宽度/8)，高度
        u8g.drawBitmapP ( 24 , 0 , 2 , 16 , bitmap_radio2); 
        u8g.drawBitmapP ( 40, 0 , 2 , 16 , bitmap_radio3); 
      }
      else{
        u8g.drawBitmapP ( 8 , 0 , 2 , 16 , bitmap_tv1); //参数：x,y,横向字节数(宽度/8)，高度
        u8g.drawBitmapP ( 24 , 0 , 2 , 16 , bitmap_tv2); 
        u8g.drawBitmapP ( 40, 0 , 2 , 16 , bitmap_tv3); 
      }
      if(actionId==0){
         u8g.drawBitmapP ( 40 , 14 , 6 , 48 , bitmap_unknown); 
      }
      else if(actionId==1){
         u8g.drawBitmapP ( 40 , 14 , 6 , 48 , bitmap_left); 
      }
      else if(actionId==2){
         u8g.drawBitmapP ( 40 , 14 , 6 , 48 , bitmap_right); 
      }
      else if(actionId==3){
         u8g.drawBitmapP ( 40 , 14 , 6 , 48 , bitmap_up); 
      }
      else if(actionId==4){
         u8g.drawBitmapP ( 40 , 14 , 6 , 48 , bitmap_down); 
      }
      else if(actionId==5){
         u8g.drawBitmapP ( 16 , 14 , 6 , 48 , bitmap_kai); 
         u8g.drawBitmapP ( 64 , 14 , 6 , 48 , bitmap_guan); 
      }
      
    } while( u8g.nextPage() );
  }
 
/**
   将识别出来的手势动作转换为红外信号发射
*/
void sendAction(int actionId) {
  Serial.print("Action:");Serial.println(ACTION_NAMES[actionId]);
  irsend.sendNEC(IR_CODES[actionId], 32);
  draw(actionId);  
}
//  const String ACTION_NAMES[9]={"Invalid","Left","Right","Up","Down","LeftUp","LeftDown","RightUp","RightDown"};
//直接根据Peak时间进行判断
int recognizeActionByPeakTime(){
   int action=0;
   if(sensors[0].peakTime==0||sensors[1].peakTime==0||sensors[2].peakTime==0||sensors[3].peakTime==0){
      action=0; 
   }
   else if(sensors[3].peakTime<sensors[0].peakTime && sensors[3].peakTime<sensors[2].peakTime
        && sensors[1].peakTime<sensors[0].peakTime && sensors[1].peakTime<sensors[2].peakTime){
      action=1;//向左
   }
   else if(sensors[3].peakTime>sensors[0].peakTime && sensors[3].peakTime>sensors[2].peakTime
        && sensors[1].peakTime>sensors[0].peakTime && sensors[1].peakTime>sensors[2].peakTime){
      action=2;//向右
   }
   else if(sensors[2].peakTime<sensors[0].peakTime && sensors[2].peakTime<sensors[1].peakTime
        && sensors[3].peakTime<sensors[0].peakTime && sensors[3].peakTime<sensors[1].peakTime){
      action=3;//向上
   }
   else if(sensors[2].peakTime>sensors[0].peakTime && sensors[2].peakTime>sensors[1].peakTime
        && sensors[3].peakTime>sensors[0].peakTime && sensors[3].peakTime>sensors[1].peakTime){
      action=4;//向下
   }
  Serial.println(String("peak:")+action);
   return action;
}
//直接根据Exit时间进行判断
int recognizeActionByExitTime(){
   int action=0;
   if(sensors[0].exitTime==0||sensors[1].exitTime==0||sensors[2].exitTime==0||sensors[3].exitTime==0){
      action=0; 
   }
   else if(sensors[3].exitTime<sensors[0].exitTime && sensors[3].exitTime<sensors[2].exitTime
        && sensors[1].exitTime<sensors[0].exitTime && sensors[1].exitTime<sensors[2].exitTime){
      action=1;//向左
   }
   else if(sensors[3].exitTime>sensors[0].exitTime && sensors[3].exitTime>sensors[2].exitTime
        && sensors[1].exitTime>sensors[0].exitTime && sensors[1].exitTime>sensors[2].exitTime){
      action=2;//向右
   }
   else if(sensors[2].exitTime<sensors[0].exitTime && sensors[2].exitTime<sensors[1].exitTime
        && sensors[3].exitTime<sensors[0].exitTime && sensors[3].exitTime<sensors[1].exitTime){
      action=3;//向上
   }
   else if(sensors[2].exitTime>sensors[0].exitTime && sensors[2].exitTime>sensors[1].exitTime
        && sensors[3].exitTime>sensors[0].exitTime && sensors[3].exitTime>sensors[1].exitTime){
      action=4;//向下
   }
  Serial.println(String("exit:")+action);
   return action;
}

//直接根据Entry时间进行判断
int recognizeActionByEntryTime(){
   int action=0;
   if(sensors[0].entryTime==0||sensors[1].entryTime==0||sensors[2].entryTime==0||sensors[3].entryTime==0){
      action=0; 
   }
   else if(sensors[3].entryTime<sensors[0].entryTime && sensors[3].entryTime<sensors[2].entryTime
        && sensors[1].entryTime<sensors[0].entryTime && sensors[1].entryTime<sensors[2].entryTime){
      action=1;//向左
   }
   else if(sensors[3].entryTime>sensors[0].entryTime && sensors[3].entryTime>sensors[2].entryTime
        && sensors[1].entryTime>sensors[0].entryTime && sensors[1].entryTime>sensors[2].entryTime){
      action=2;//向右
   }
   else if(sensors[2].entryTime<sensors[0].entryTime && sensors[2].entryTime<sensors[1].entryTime
        && sensors[3].entryTime<sensors[0].entryTime && sensors[3].entryTime<sensors[1].entryTime){
      action=3;//向上
   }
   else if(sensors[2].entryTime>sensors[0].entryTime && sensors[2].entryTime>sensors[1].entryTime
        && sensors[3].entryTime>sensors[0].entryTime && sensors[3].entryTime>sensors[1].entryTime){
      action=4;//向下
   }
  Serial.println(String("entry:")+action);
   return action;
}
//根据三个不同类型的时间进行判断
int recognizeAction(){
  //先检测是否为开关动作
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensors[i].exitTime-sensors[i].entryTime >1000) {
      return 5;
    }
  }
  int action1=recognizeActionByPeakTime();
  int action2=recognizeActionByEntryTime();
  int action3=recognizeActionByExitTime();
  //如果有两个action相同，直接返回
  if(action1!=0&&action1==action2){
    return action1;
  }
  if(action2!=0&&action2==action3){
    return action2;
  }
  if(action3!=0&&action3==action1){
    return action3;
  }
  //返回单个action
  if(action1!=0){
    return action1;
  }
  if(action2!=0){
    return action2;
  }
  if(action3!=0){
    return action3;
  }
  return 0;
}
void resetSensorData() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].curV = 0;
    sensors[i].peakV = 0;
    sensors[i].peakTime = 0;
    sensors[i].entryTime = 0;
    sensors[i].exitTime = 0;
  }
}

void processButtonState(){
    // 读取按钮状态
  int newButtonState = digitalRead(buttonPin);
  if(buttonState==LOW && newButtonState==HIGH){
    //按钮被按下，切换当前功能
    functionType=functionType+1;
    if(functionType>=2){
        functionType=0;//回到最初功能
    }
    draw(0);//重新显示功能状态
    debugCount=0;//重置debug计数器
  }
  buttonState=newButtonState;
}

void loop() {
  //读取按钮状态
  processButtonState();
  
  readSensorData();
if(false){
  //printSensorData(1,"",sensors);
  printSimpleSensorData(sensors);
  delay(1);
  return;
}
  if (hasCompletedAction()) {
    Serial.println("New Action");
    printSensorData(0,"",sensors);
    int action = recognizeAction();
    sendAction(action);
    resetSensorData();
    delay(1000);
  }
  unsigned long t = millis();
  if(t-lastDebugTime>5000){
    Serial.println("No Action.");
    resetSensorData();
    lastDebugTime=t;
  }
    delay(1);

}

