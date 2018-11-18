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
const int ACTION_COUNT=5;
//将时间分为3等分
const int ACTION_CODES[ACTION_COUNT][7]={//手势必须覆盖所有传感器
                    {9999},  //Invalid
                    {3131},/*3132,3231,2131,3121,2231,1322}, /* LEFT    31  31 32 21 31
                                                                     31  32 31 31 21*/
                    {1313},/*1213,1312,1323,2313,2213,3122}, /* RIGHT   13  12 13 13 23
                                                                     13  13 12 23 13*/
                    {3311},/*2311,3211,3312,3321,2312,2321}, /* UP      33  23 32 33 33
                                                                     11  11 11 12 21*/
                    {1133},/*1123,1132,1233,2133,1232,2123} /* DOWN     11  11 11 12 21
                                                                     33  23 32 33 33 */
            };

//上次打印debug时间，用于控制debug输出内容的时间间隔
unsigned long lastDebugTime=0;

//排序用时间数组
struct Sensor {
  int sensorId;//传感器对应下标
  float curV;//当前电压
  float avgV;//平均电压
  float peakV;//最大电压
  unsigned long peakVTime;//传感器最大波峰时间(ms)
  unsigned long entryTime;//传感器进入波峰时间(ms)
  int peakOrder;//传感器所在时间段顺序，按波峰排序
  int entryOrder;//传感器所在时间段顺序，按进入波分排序
};

Sensor sensors[SENSOR_COUNT];

void initVArray() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].sensorId = i;
    sensors[i].curV = 0;
    sensors[i].avgV = 0;
    sensors[i].peakV = 0;
    sensors[i].peakVTime = 0;
    sensors[i].entryTime = 0;
    sensors[i].peakOrder = 0;
    sensors[i].entryOrder = 0;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Let's go!");
  initVArray();
}
//判断当前电压是否高出平均值
int isAboveAvgV(Sensor s){
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
       sensors[s].entryTime = 0;
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
    if(isAboveAvgV(sensors[s])){
      if(sensors[s].entryTime==0){
        sensors[s].entryTime=t;
      }
      if (sensors[s].curV > sensors[s].peakV   ) {
        sensors[s].peakV = sensors[s].curV;
        sensors[s].peakVTime = t;
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
    int sensorId=sensors[i].sensorId;
    Serial.println(SENSOR_NAMES[sensorId]+"  sensorId:"+sensorId+"  cur:"+(int)sensors[i].curV+"  peakV:"+(int)sensors[i].peakV+"  avgV:"+(int)sensors[i].avgV
    +"  peakVTime:" +(sensors[i].peakVTime/1000)+(".")+(sensors[i].peakVTime % 1000)
    +"  entryTime:" +(sensors[i].entryTime/1000)+(".")+(sensors[i].entryTime % 1000)
    +"  peakOrder:"+sensors[i].peakOrder
    +"  entryOrder:"+sensors[i].entryOrder
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
    if (isAboveAvgV(sensors[s])==1) {
      hasPeak=1;
      break;
    }
  }
  if(hasPeak==1){
    return 0;
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
//按照波峰时间排序
void recognizeAction_sort1() {
   //计算手势的开始/结束时间
  unsigned long firstTime=0;
  unsigned long lastTime =0;
  for (int i = 1; i < SENSOR_COUNT; i++) {
    unsigned long t=sensors[i].peakVTime;
    if (t > 0) {
      if(firstTime==0||t<firstTime){//firstTime未设置或者当前t小于firstTime
        firstTime=t;
      }
      if(lastTime==0||t>lastTime){//lastTime未设置或者当前t大于lastTime
        lastTime=t;
      }
    }
  }
  
  //将整个手势的时间分为3等分，计算每个传感器所在时间段
  double totalTime=lastTime-firstTime;
  for(int i=0;i<SENSOR_COUNT;i++){
    double curTimes=sensors[i].peakVTime-firstTime;//当前传感器出现peakVTime的时间延迟
    double rate=curTimes/totalTime;//当前传感器延迟与总时间的比例
    if(rate<0.40){//计算每个传感器序号
      sensors[i].peakOrder=1;
    }
    else if(rate<0.60){
      sensors[i].peakOrder=2;
    }
    else{
      sensors[i].peakOrder=3;
    }
  }
}
//按照进入波峰时间排序
void recognizeAction_sort2() {
   //计算手势的开始/结束时间
  unsigned long firstTime=0;
  unsigned long lastTime =0;
  for (int i = 1; i < SENSOR_COUNT; i++) {
    unsigned long t=sensors[i].entryTime;
    if (t > 0) {
      if(firstTime==0||t<firstTime){//firstTime未设置或者当前t小于firstTime
        firstTime=t;
      }
      if(lastTime==0||t>lastTime){//lastTime未设置或者当前t大于lastTime
        lastTime=t;
      }
    }
  }
  
  //将整个手势的时间分为3等分，计算每个传感器所在时间段
  double totalTime=lastTime-firstTime;
  for(int i=0;i<SENSOR_COUNT;i++){
    double curTimes=sensors[i].entryTime-firstTime;//当前传感器出现peakVTime的时间延迟
    double rate=curTimes/totalTime;//当前传感器延迟与总时间的比例
    if(rate<0.4){//计算每个传感器序号
      sensors[i].entryOrder=1;
    }
    else if(rate<0.6){
      sensors[i].entryOrder=2;
    }
    else{
      sensors[i].entryOrder=3;
    }
  }
}

//将排序结果转换为整数类型的action，
int recognizeAction_covertToActionId(){
  int actionCode=0;
  //先按照进入波峰时间查找action
  for(int i=0;i<SENSOR_COUNT;i++){
    actionCode=actionCode*10+sensors[i].entryOrder;
  }
  Serial.println(String("entry:")+actionCode);
  for(int i=0;i<ACTION_COUNT;i++){
      for(int j=0;j<7;j++){
        if(ACTION_CODES[i][j]==actionCode){
          return i;
      }
    }
  }
  actionCode=0;
  //按照波峰时间查找action
  for(int i=0;i<SENSOR_COUNT;i++){
    actionCode=actionCode*10+sensors[i].peakOrder;
  }
  Serial.println(String("peak:")+actionCode);
  for(int i=0;i<ACTION_COUNT;i++){
      for(int j=0;j<7;j++){
        if(ACTION_CODES[i][j]==actionCode){
          return i;
      }
    }
  }
  return 0;//如果未找到，return 0
}
/**
   根据SensorData识别Action
*/
int recognizeAction() {
  recognizeAction_sort1();
  recognizeAction_sort2();
  printSensorData(1,"Sort Result",sensors);
  return recognizeAction_covertToActionId();
}

void resetSensorData() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].curV = 0;
    sensors[i].peakV = 0;
    sensors[i].peakVTime = 0;
    sensors[i].entryTime = 0;
    sensors[i].peakOrder = 0;
    sensors[i].entryOrder = 0;
  }
}

void loop() {
  readSensorData();
//  printSensorData(1,"",sensors);
  if (hasCompletedAction()) {
    Serial.println("New Action");
   // printSensorData(0,"",sensors);
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

