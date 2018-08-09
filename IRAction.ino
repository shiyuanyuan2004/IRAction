//sensor重命名
#include "Arduino.h"
//不要使用自定义format函数，函数内字符串拼接有bug，导致死机
/*
 * 红外传感器数目，2*2
 * 传感器位置
 *  S0   S1
 *  S2   S3 
 */
const int SENSOR_COUNT = 4;
const int ACTION_COUNT=9;
const int ACTION_CODE_COUNT=7;
const int TIME_SPLITS=6;//将整个action时间平分为6份,再多的话训练数据量太多
const String ACTION_NAMES[9]={"Invalid","Left","Right","Up","Down","LeftUp","LeftDown","RightUp","RightDown"};
const int ACTION_CODES[ACTION_COUNT][ACTION_CODE_COUNT]={
                    {9999}, //Invalid
                    {2199,9921,2121,/*2131,3121,3132,3231*/},/* LEFT    21 99 21 (容错) 21 31 31 32
                                                                    99 21 21 (容错) 31 21 32 31*/
                    {1299,9912,1212,/*1323,2313,1213,1312*/},/* RIGHT   12 99 12 (容错) 13 23 12 13
                                                                    99 12 12 (容错) 23 13 13 12 */
                    {2919,9291,2211,/*2311,3211,3312,3321*/},/* UP      29 92 22 (容错) 23 32 33 33
                                                                    19 91 11 (容错) 11 11 12 21  */
                    {1929,9192,1122,/*1123,1132,1233,2133*/},/* DOWN    19 91 11 (容错) 11 11 12 21
                                                                    29 92 22 (容错) 23 32 33 33  */
                    {3221,4321,4231,3921,3291},/* LEFT_UP    32 43 42 39 32
                                                             21 21 31 21 91  */
                    {2132,2143,3142,2139,9132},/* LEFT_DOWN  21 21 31 21 91
                                                             32 43 42 39 32  */
                    {2312,2319,9312,3412,2413},/* RIGHT_UP   23 23 93 34 24
                                                             12 19 12 12 13  */
                    {1324,1234,1223,1293,1923} /*RIGHT_DOWN  13 12 12 12 19
                                                             24 34 23 93 23  */
            };
//传感器输入pin，数组下标按照从左到右、从上到下方式排列
const int INPUT_PIN[SENSOR_COUNT] = {A1, A2, A3, A4};
const String SENSOR_NAMES[SENSOR_COUNT] = {"S_1_1", "S_1_2", "S_2_1", "S_2_2"};
//每次循环延迟时间(ms)，要求小于5ms，太短可能提高cpu使用率
const int LOOP_DELAY = 1;

//计算平均电压的时间区间，转换为sample次数,设置为100秒，该值只影响取平均值的平滑度
const float AVG_V_SAMPLE_COUNT = 100.0f * 1000 / LOOP_DELAY;
//手势动作最大的时间区间，超过这个时间的波峰将被忽略掉
const long PEAK_ACTION_PERIOD=3000;
//波峰电压只要需要高出平均电压多少
float MIN_PEAK_CUR_V_RATE = 1.1f;

//不同传感器之间取样间隔
const int SAMPLE_DELAY = 0;



//判断波峰的最小值，相对于平均值的比例
const float MIN_PEAK_V_RATE = 1.2f;
//判断波峰的最小值，相对于平均值增加的数量
const int MIN_PEAK_V_AMOUNT = 50;

//上次打印debug时间，用于控制debug输出内容的时间间隔
unsigned long lastDebugTime=0;

//排序用时间数组
struct Sensor {
  int sensorId;//传感器对应下标
  float curV;//当前电压
  float avgV;//平均电压
  float peakV;//最大电压
  unsigned long peakVTime;//传感器最大波峰时间(ms)
  int order;//实际顺序，两个传感器序号可能相同。最早的传感器为1，如果某个传感器没有波峰，则设置为9
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
//判断当前电压是否高出平均值的阈值，满足PeakV基本条件
int isAboveAvgV(Sensor &s){
 if( s.curV > s.avgV*MIN_PEAK_V_RATE&&s.curV>s.avgV+MIN_PEAK_V_AMOUNT){
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
  for (int s = 0; s < SENSOR_COUNT; s++) {
    //清理掉PEAK_ACTION_PERIOD之前的数据
    if(sensors[s].peakV>0&&t-sensors[s].peakVTime>PEAK_ACTION_PERIOD){
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
    if (sensors[s].curV > sensors[s].peakV
    && isAboveAvgV(sensors[s])) {
      sensors[s].peakV = sensors[s].curV;
      sensors[s].peakVTime = t;
    }
  }
}


void printSensorData(int printInfo,String info,Sensor sensors[]){
  if(printInfo!=0){
      Serial.println(String("***********  ")+info+"  ***********");
  }
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
  Serial.print("Action:");Serial.println(ACTION_NAMES[actionId]);
}
/**
 * 按peak进行排序
 */
void sort(){
   //获取开始/结束时间
  unsigned long startTime=sensors[0].peakVTime;
  unsigned long endTime =sensors[0].peakVTime;
  for (int i = 1; i < SENSOR_COUNT; i++) {
    unsigned long peakTime=sensors[i].peakVTime;
    if (peakTime > 0) {
        if(peakTime<startTime){
          startTime=peakTime;
        }
        if(peakTime>endTime){
          endTime=peakTime;
        }
    }
  }
  //排序
  long totalTime=endTime-startTime;
  Serial.print("startTime:");Serial.print(startTime);Serial.print(",endTime:");Serial.print(endTime);Serial.print(",totalTime:");Serial.println(totalTime);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    //根据时间偏移，除以分片时间，直接获取序号，从1开始
    int order;
    if(sensors[i].peakVTime==0){
      order=0;
    }
    else{
      long timeOffset=sensors[i].peakVTime-startTime;
      order=timeOffset*TIME_SPLITS/totalTime+1;
      if(order>TIME_SPLITS){
        order=TIME_SPLITS;//最后一个设置为TIME_SPLITS
      }
    }
    sensors[i].order = order;
  }
}
//将排序结果转换为整数类型的action，
int covertToActionId(){
  int actionCode=0;
  for(int i=0;i<SENSOR_COUNT;i++){
    actionCode=actionCode*10+sensors[i].order;
  }
  
  Serial.println(actionCode);
  for(int i=0;i<ACTION_COUNT;i++){
    for(int j=0;j<ACTION_CODE_COUNT;j++){
      if(ACTION_CODES[i][j]==actionCode){
        return i;
      }
    }
  }
  return 0;//如果未找到，return 0
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
  if (hasCompletedAction()) {
    Serial.println("New Action");
    sort();
    printSensorData(0,"",sensors);
    int action =covertToActionId();
    sendAction(action);
    resetSensorData();
  }
  unsigned long t = millis();
  if(t-lastDebugTime>2000){
    Serial.println("No Action.");
    lastDebugTime=t;
  }
  //delay(LOOP_DELAY);
    delay(1);
}

