//sensor重命名
#include "Arduino.h"
#include <IRremote.h>


//不要使用自定义format函数，函数内字符串拼接有bug，导致死机
/*
 * 红外传感器数目，2*2
 * 传感器位置
 *  S1_1   S1_2
 *  S2_1   S2_2 
 */

const int SENSOR_COUNT = 4;
const int ACTION_COUNT=5;
const String ACTION_NAMES[5]={"Invalid","Left","Right","Up","Down"};
//上次打印debug时间，用于控制debug输出内容的时间间隔
unsigned long lastDebugTime=0;
//红外遥控相关常亮
int RECV_PIN = 11;
//IRrecv irrecv(RECV_PIN);
IRsend irsend;
const long IR_STOP_RESUME=0xFF02FD;
const long IR_NEXT=0xFFF00F;
const long IR_PREVIOUS=0xFFA05F;
const long IR_VOL_UP=0xFF40BF;
const long IR_VOL_DOWN=0xFFE01F;

long IR_CODES[6]={0,IR_PREVIOUS,IR_NEXT,IR_VOL_UP,IR_VOL_DOWN};
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
  Serial.begin(115200);
  pinMode(2, OUTPUT);      // sets the digital pin as output
  pinMode(3, OUTPUT);      // sets the digital pin as output
  pinMode(4, OUTPUT);      // sets the digital pin as output
  pinMode(5, OUTPUT);      // sets the digital pin as output

  Serial.println("Let's go!");
  initVArray();
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
  const int INPUT_PIN[SENSOR_COUNT] = {A1, A2, A3, A4};
  
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

/**
   将识别出来的手势动作转换为红外信号发射
*/
void sendAction(int actionId) {
  Serial.print("Action:");Serial.println(ACTION_NAMES[actionId]);
  digitalWrite(2, LOW);   // turn the LED off
  digitalWrite(3, LOW);   // turn the LED off
  digitalWrite(4, LOW);   // turn the LED off
  digitalWrite(5, LOW);   // turn the LED off
  digitalWrite(actionId+1, HIGH);   // turn the LED on

  delay(40);
  irsend.sendNEC(IR_CODES[actionId], 32);
  delay(1000);
  
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

void loop() {
  readSensorData();
//  printSensorData(1,"",sensors);
  if (hasCompletedAction()) {
    Serial.println("New Action");
    printSensorData(0,"",sensors);
 //   int action = recognizeAction();
    int action = recognizeAction();
    sendAction(action);
    resetSensorData();
  }
  unsigned long t = millis();
  if(t-lastDebugTime>5000){
    Serial.println("No Action.");
    lastDebugTime=t;
  }
    delay(1);
}

