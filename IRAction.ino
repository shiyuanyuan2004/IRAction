//sensor重命名
#include "Arduino.h"
//不要使用自定义format函数，函数内字符串拼接有bug，导致死机
/*
 * 红外传感器数目，2*2
 * 传感器位置
 *  S1_1   S1_2
 *  S2_1   S2_2 
 */
const int SENSOR_COUNT = 4;
const int ACTION_COUNT=9;
//将时间分为3等分
const int ACTION_CODES[ACTION_COUNT]={//手势必须覆盖所有传感器
                    9999,  //Invalid
                    3131,  /* LEFT    31
                                      31  */
                    1313,  /* RIGHT   13
                                      13  */
                    3311,  /* UP      33
                                      11  */
                    1133,  /* DOWN    11
                                      33  */
                    3221,  /* LEFT_UP    32
                                         21  */
                    2132,  /* LEFT_DOWN  21 
                                         32  */
                    2312,  /* RIGHT_UP   23 
                                         12  */
                    1223   /*RIGHT_DOWN  12
                                         23  */
            };


//手势动作最大的时间区间，超过这个时间的波峰将被忽略掉

//上次打印debug时间，用于控制debug输出内容的时间间隔
unsigned long lastDebugTime=0;

//排序用时间数组
struct Sensor {
  int sensorId;//传感器对应下标
  float curV;//当前电压
  float avgV;//平均电压
  float peakV;//最大电压
  unsigned long peakVTime;//传感器最大波峰时间(ms)
  int order;//传感器所在时间段顺序
};

Sensor sensors[SENSOR_COUNT];

void initVArray() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].sensorId = i;
    sensors[i].curV = 0;
    sensors[i].avgV = 0;
    sensors[i].peakV = 0;
    sensors[i].peakVTime = 0;
    sensors[i].order = 0;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Started!");
  initVArray();
}
//判断当前电压是否高出平均值
int isAboveAvgV(Sensor &s){
  //判断波峰的最小值必须是平均值的1.5倍，并且要比平均值大50以上，避免误差
 if( s.curV > s.avgV*1.2f&&s.curV>s.avgV+50){
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
  
  //计算最近100秒平均电压所需的取样次数，该值只影响取平均值的平滑度
  const float AVG_V_SAMPLE_COUNT = 100.0f * 1000 ;

  for (int s = 0; s < SENSOR_COUNT; s++) {
    //清理掉3秒之前的数据，一个手势动作的最长时间不应该超过3秒
     if(sensors[s].peakV>0&&t-sensors[s].peakVTime>3000){
       sensors[s].peakV = 0;
       sensors[s].peakVTime = 0;
    }
    //读取数据
    sensors[s].curV = analogRead(INPUT_PIN[s]);
    if(sensors[s].avgV<0.1){
      sensors[s].avgV=sensors[s].curV;
    }
    else{
      sensors[s].avgV = (sensors[s].avgV * (AVG_V_SAMPLE_COUNT - 1) + sensors[s].curV) / AVG_V_SAMPLE_COUNT;
    }
    //电压超过平均值的y%，才设置波峰
    if (sensors[s].curV > sensors[s].peakV  && isAboveAvgV(sensors[s])) {
      sensors[s].peakV = sensors[s].curV;
      sensors[s].peakVTime = t;
    }
  }
}

void printSensorData(int printInfo,String info,Sensor sensors[]){
  if(printInfo!=0){
      Serial.println(String("***********  ")+info+"  ***********");
  }
  const String SENSOR_NAMES[SENSOR_COUNT] = {"S_1_1", "S_1_2", "S_2_1", "S_2_2"};
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int sensorId=sensors[i].sensorId;
    Serial.println(SENSOR_NAMES[sensorId]+"   sensorId:"+sensorId+"   cur:"+sensors[i].curV+"   peakV:"+sensors[i].peakV+"   avgV:"+sensors[i].avgV+"   peakVTime:"
    +(sensors[i].peakVTime/1000)+(".")+(sensors[i].peakVTime % 1000)
    +"   order:"+sensors[i].order);
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
    if (isAboveAvgV(sensors[s])) {
      hasPeak=1;
      break;
    }
  }
  if(hasPeak==1){
    return;
  }
  
  //至少有一个Sensor有peak值
  for (int s = 0; s < SENSOR_COUNT; s++) {
    if (sensors[s].peakV >0.1) {
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
  const String ACTION_NAMES[9]={"Invalid","Left","Right","Up","Down","LeftUp","LeftDown","RightUp","RightDown"};
  Serial.print("Action:");Serial.println(ACTION_NAMES[actionId]);
}

int recognizeAction_sort() {
   //计算手势的开始/结束时间
  unsigned long firstTime=0;
  unsigned long lastTime =0;
  for (int i = 1; i < SENSOR_COUNT; i++) {
    unsigned long peakTime=sensors[i].peakVTime;
    if (peakTime > 0) {
      if(firstTime==0||peakTime<firstTime){//firstTime未设置或者当前peakTime小于firstTime
        firstTime=peakTime;
      }
      if(lastTime==0||peakTime>lastTime){//lastTime未设置或者当前peakTime大于lastTime
        lastTime=peakTime;
      }
    }
  }
  
  //将整个手势的时间分为3等分，计算每个传感器所在时间段
  unsigned long totalTime=lastTime-firstTime;
  for(int i=0;i<SENSOR_COUNT-1;i++){
    unsigned long curTimes=sensors[i].peakVTime-firstTime;//当前传感器出现peakVTime的时间延迟
    double rate=curTimes/totalTime;//当前传感器延迟与总时间的比例
    if(rate<0.333){//计算每个传感器序号
      sensors[i].order=1;
    }
    else if(rate<0.666){
      sensors[i].order=2;
    }
    else{
      sensors[i].order=3;
    }
  }
}

//将排序结果转换为整数类型的action，
int recognizeAction_covertToActionId(){
  int actionCode=0;
  for(int i=0;i<SENSOR_COUNT;i++){
    actionCode=actionCode*10+sensors[i].order;
  }

  Serial.println(actionCode);
  for(int i=0;i<ACTION_COUNT;i++){
      if(ACTION_CODES[i]==actionCode){
        return i;
    }
  }
  return 0;//如果未找到，return 0
}
/**
   根据SensorData识别Action
*/
int recognizeAction() {
  recognizeAction_sort();
  printSensorData(1,"Sort Result",sensors);
  resetSensorData();
  return recognizeAction_covertToActionId();
}

void resetSensorData() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].curV = 0;
    sensors[i].peakV = 0;
    sensors[i].peakVTime = 0;
    sensors[i].order = 0;
  }
}

void loop() {
  readSensorData();
  //printSensorData(1,"",sensors);
  if (hasCompletedAction()) {
    Serial.println("New Action");
    printSensorData(0,"",sensors);
    int action = recognizeAction();
    sendAction(action);
    resetSensorData();
  }
  unsigned long t = millis();
  if(t-lastDebugTime>2000){
    Serial.println("No Action.");
    lastDebugTime=t;
  }
    delay(1);
}

