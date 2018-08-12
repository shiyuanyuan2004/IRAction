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
const int ACTION_CODE_COUNT=16;
const int TIME_SPLITS=99;//将整个action时间平分为99份
const double H_V_OFFSET_RATE=0.20;//水平、垂直方向的最大角度偏差
const String ACTION_NAMES[9]={"Invalid","Left","Right","Up","Down","LeftUp","LeftDown","RightUp","RightDown"};
const int ACTION_INVALID=0,ACTION_LEFT=1,ACTION_RIGHT=2,ACTION_UP=3,ACTION_DOWN=4,ACTION_LEFT_UP=5,ACTION_LEFT_DOWN=6,ACTION_RIGHT_UP=7,ACTION_RIGHT_DOWN=8;
const int S11=0,S12=1,S21=2,S22=3;
//传感器输入pin，数组下标按照从左到右、从上到下方式排列
const int INPUT_PIN[SENSOR_COUNT] = {A1, A2, A3, A4};
const String SENSOR_NAMES[SENSOR_COUNT] = {"S_1_1", "S_1_2", "S_2_1", "S_2_2"};
//每次循环延迟时间(ms)，要求小于5ms，太短可能提高cpu使用率
const int LOOP_DELAY = 1;

//计算平均电压的时间区间，转换为sample次数,设置为100秒，该值只影响取平均值的平滑度
const float AVG_V_SAMPLE_COUNT = 100.0f * 1000 / LOOP_DELAY;
//手势动作最大的时间区间，超过这个时间的波峰将被忽略掉
const uint64_t PEAK_ACTION_PERIOD=3000000;
//波峰电压只要需要高出平均电压多少
float MIN_PEAK_CUR_V_RATE = 1.1f;

//不同传感器之间取样间隔
const int SAMPLE_DELAY = 0;
//判断波峰的最小值，相对于平均值的比例
const float MIN_PEAK_V_RATE = 1.2f;
//判断波峰的最小值，相对于平均值增加的数量
const int MIN_PEAK_V_AMOUNT = 50;

//上次打印debug时间，用于控制debug输出内容的时间间隔
uint64_t lastDebugTime=0;
const int ENTRY=0;
const int PEAK=1;
const int EXIT=2;
//排序用时间数组
struct Sensor {
  int sensorId;//传感器对应下标
  float curV;//当前电压
  float avgV;//平均电压
  float peakV;//最大电压
  uint64_t peakVTime[3];//传感器波峰时间(微秒),Entry/Peak/Exit
  int peakOrder[3];//波峰顺序,Entry/Peak/Exit，两个传感器序号可能相同。最早的传感器为1，如果某个传感器没有波峰，则设置为0
  //使用entry/peak/exit三个时间和顺序，通过三套方式进行判断，提高准确度
};
Sensor sensors[SENSOR_COUNT];
void resetSensorData() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].curV = 0;
    sensors[i].peakV = 0;
    for(int n=0;n<=2;n++){
      sensors[i].peakVTime[n] = 0;
      sensors[i].peakOrder[n] = 0;
    }
  }
}

void initVArray() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].sensorId = i;
    sensors[i].curV = 0;
    sensors[i].avgV = 0;
    sensors[i].peakV = 0;
    for(int n=0;n<=2;n++){
      sensors[i].peakVTime[n]=0;
      sensors[i].peakOrder[n]=0;
    }
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
  uint64_t t = micros();
  //  shiftSensorData();
  for (int s = 0; s < SENSOR_COUNT; s++) {
    //清理掉PEAK_ACTION_PERIOD之前的数据
    if(sensors[s].peakV>0&&t-sensors[s].peakVTime[PEAK]>PEAK_ACTION_PERIOD){
       sensors[s].peakV = 0;
       for(int i=0;i<=2;i++){
        sensors[s].peakVTime[i] = 0;
       }
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
      if (sensors[s].curV > sensors[s].peakV ) {//设置波峰
        sensors[s].peakV = sensors[s].curV;
        sensors[s].peakVTime[PEAK] = t;
      }
      if (sensors[s].peakVTime[ENTRY]==0 ) {//设置进入时间
        sensors[s].peakVTime[ENTRY] = t;
      }
    }
    else{
      if (sensors[s].peakVTime[EXIT]==0&&sensors[s].peakVTime[ENTRY]>0 ) {//设置离开时间
        sensors[s].peakVTime[EXIT] = t;
      }
    }
  }
}


void printSensorData(int printInfo,String info,Sensor sensors[]){
  if(printInfo!=0){
      Serial.println(String("***********  ")+info+"  ***********");
  }
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int sensorId=sensors[i].sensorId;
    Serial.println(SENSOR_NAMES[sensorId]+"   sensorId:"+sensorId+"   cur:"+sensors[i].curV+"   peakV:"+sensors[i].peakV+"   avgV:"+sensors[i].avgV
      +"   entryTime:" +((double)sensors[i].peakVTime[ENTRY]/1000)+"   entryOrder:"+sensors[i].peakOrder[ENTRY]
      +"   peakVTime:" +((double)sensors[i].peakVTime[PEAK]/1000)+"   peakOrder:"+sensors[i].peakOrder[PEAK]
      +"   exitTime:" +((double)sensors[i].peakVTime[EXIT]/1000)+"   exitOrder:"+sensors[i].peakOrder[EXIT]
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
  for(int n=0;n<=2;n++){//分别排序Entry/Peak/Exit
     //获取开始/结束时间
    uint64_t startTime=sensors[0].peakVTime[n];
    uint64_t endTime =sensors[0].peakVTime[n];
    for (int i = 1; i < SENSOR_COUNT; i++) {
      uint64_t peakTime=sensors[i].peakVTime[n];
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
    uint64_t totalTime=endTime-startTime;
  //  Serial.print("startTime:");Serial.print(startTime);Serial.print(",endTime:");Serial.print(endTime);Serial.print(",totalTime:");Serial.println(totalTime);
    if(totalTime==0){
      continue;
    }
    for (int i = 0; i < SENSOR_COUNT; i++) {
      //根据时间偏移，除以分片时间，直接获取序号，从1开始
      int order;
      if(sensors[i].peakVTime==0){
        order=0;
      }
      else{
        uint64_t timeOffset=sensors[i].peakVTime[n]-startTime;
        order=timeOffset*TIME_SPLITS/totalTime+1;
      }
      sensors[i].peakOrder[n] = order;
    }
  }
}
//将排序结果转换为整数类型的action，
int covertToActionId(){
  int actions[3]={ACTION_INVALID};
  int offset=TIME_SPLITS*H_V_OFFSET_RATE;
  Serial.print("Offset:");Serial.println(offset);
  for(int n=0;n<=2;n++){
//const int ACTION_INVALID=0,ACTION_LEFT=1,ACTION_RIGHT=2,ACTION_UP=3,ACTION_DOWN=4,ACTION_LEFT_UP=5,ACTION_LEFT_DOWN=6,ACTION_RIGHT_UP=7,ACTION_RIGHT_DOWN=8;
    if(sensors[S11].peakOrder[n]< offset && sensors[S21].peakOrder[n]< offset  
      &sensors[S12].peakOrder[n]>TIME_SPLITS-offset && sensors[S22].peakOrder[n]>TIME_SPLITS-offset){
        //左边两个传感器在前offset时间段，右边两个传感器在后offset时间段，
        actions[n]=ACTION_RIGHT;
    }
    if(sensors[S12].peakOrder[n]< offset && sensors[S22].peakOrder[n]< offset  
      &sensors[S11].peakOrder[n]>TIME_SPLITS-offset && sensors[S21].peakOrder[n]>TIME_SPLITS-offset){
        //右边两个传感器在前offset时间段，左边两个传感器在后offset时间段，
        actions[n]=ACTION_LEFT;
    }
    if(sensors[S11].peakOrder[n]< offset && sensors[S12].peakOrder[n]< offset  
      &sensors[S21].peakOrder[n]>TIME_SPLITS-offset&& sensors[S22].peakOrder[n]>TIME_SPLITS-offset){
        //上边两个传感器在前offset时间段，下边两个传感器在后offset时间段，
        actions[n]=ACTION_DOWN;
    }
    if(sensors[S21].peakOrder[n]< offset && sensors[S22].peakOrder[n]< offset  
      &&sensors[S11].peakOrder[n]>TIME_SPLITS-offset && sensors[S12].peakOrder[n]>TIME_SPLITS-offset){
        //下边两个传感器在前offset时间段，上边两个传感器在后offset时间段，
        actions[n]=ACTION_UP;
    }
    if(sensors[S11].peakOrder[n]< offset && sensors[S22].peakOrder[n]> TIME_SPLITS-offset 
      &&sensors[S12].peakOrder[n]>=offset && sensors[S12].peakOrder[n]<=TIME_SPLITS-offset
      &&sensors[S21].peakOrder[n]>=offset && sensors[S21].peakOrder[n]<=TIME_SPLITS-offset
      ){
        //左上传感器在前offset时间段，右下传感器在后offset时间段，其余在中间时间段
        actions[n]=ACTION_RIGHT_DOWN;
    }
    if(sensors[S22].peakOrder[n]< offset && sensors[S11].peakOrder[n]> TIME_SPLITS-offset 
      &&sensors[S12].peakOrder[n]>=offset && sensors[S12].peakOrder[n]<=TIME_SPLITS-offset
      &&sensors[S21].peakOrder[n]>=offset && sensors[S21].peakOrder[n]<=TIME_SPLITS-offset
      ){
        //右下传感器在前offset时间段，左上传感器在后offset时间段，其余在中间时间段
        actions[n]=ACTION_LEFT_UP;
    }
    if(sensors[S12].peakOrder[n]< offset && sensors[S21].peakOrder[n]> TIME_SPLITS-offset 
      &&sensors[S11].peakOrder[n]>=offset && sensors[S11].peakOrder[n]<=TIME_SPLITS-offset
      &&sensors[S22].peakOrder[n]>=offset && sensors[S22].peakOrder[n]<=TIME_SPLITS-offset
      ){
        //右上传感器在前offset时间段，左下传感器在后offset时间段，其余在中间时间段
        actions[n]=ACTION_LEFT_DOWN;
    }
    if(sensors[S21].peakOrder[n]< offset && sensors[S12].peakOrder[n]> TIME_SPLITS-offset 
      &&sensors[S11].peakOrder[n]>=offset && sensors[S11].peakOrder[n]<=TIME_SPLITS-offset
      &&sensors[S22].peakOrder[n]>=offset && sensors[S22].peakOrder[n]<=TIME_SPLITS-offset
      ){
        //左下传感器在前offset时间段，右上传感器在后offset时间段，其余在中间时间段
        actions[n]=ACTION_RIGHT_UP;
    }
    
  }
  Serial.print("actions[ENTRY]:");Serial.println(ACTION_NAMES[actions[ENTRY]]);
  Serial.print("actions[PEAK] :");Serial.println(ACTION_NAMES[actions[PEAK]]);
  Serial.print("actions[EXIT] :");Serial.println(ACTION_NAMES[actions[EXIT]]);

  //如果某个action出现两次，则返回。
  if(actions[PEAK]>0&&(actions[ENTRY]==actions[PEAK]||actions[EXIT]==actions[PEAK])){
    return actions[PEAK];
  }
  if(actions[ENTRY]>0&&(actions[PEAK]==actions[ENTRY]||actions[EXIT]==actions[ENTRY])){
    return actions[ENTRY];
  }
  if(actions[EXIT]>0&&(actions[PEAK]==actions[EXIT]||actions[EXIT]==actions[ENTRY])){
    return actions[EXIT];
  }
  //如果没有出现两次的action，则返回一次的action，按照peak/entry/exit顺序返回
  //TODO 如果有斜向的action出现，则优先返回
  if(actions[PEAK]>0){
    return actions[PEAK];
  }
  if(actions[ENTRY]>0){
    return actions[ENTRY];
  }
  if(actions[EXIT]>0){
    return actions[EXIT];
  }
  
  return 0;//如果未找到，return 0
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
  uint64_t t = micros();
  if(t-lastDebugTime>2000000){
    Serial.println("No Action.");
    lastDebugTime=t;
  }
  //delay(LOOP_DELAY);
    delayMicroseconds( 100 );
}

