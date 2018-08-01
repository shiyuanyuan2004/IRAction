sensor重命名
#include "Arduino.h"
//不要使用自定义format函数，函数内字符串拼接有bug，导致死机
//红外传感器数目，2*2
const int SENSOR_COUNT = 4;

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

const int ACTION_LEFT = 1;
const int ACTION_RIGHT = 2;
const int ACTION_UP = 3;
const int ACTION_DOWN = 4;
const int ACTION_LEFT_UP = 5;
const int ACTION_LEFT_DOWN = 6;
const int ACTION_RIGHT_UP = 7;
const int ACTION_RIGHT_DOWN = 8;


//判断波峰的最小值，相对于平均值的比例
const float MIN_PEAK_V_RATE = 1.2f;
//判断波峰的最小值，相对于平均值增加的数量
const int MIN_PEAK_V_AMOUNT = 50;

//判断两个传感器先后顺序的最小间隔时间比例，以整个手势时间为基准。用于容错处理，允许少于的方向偏差
const float MIN_SENSOR_PERIOD_RATE = 0.2f;

//上次打印debug时间，用于控制debug输出内容的时间间隔
unsigned long lastDebugTime=0;

//排序用时间数组
struct Sensor {
  int sensorId;//传感器对应下标
  float curV;//当前电压
  float avgV;//平均电压
  float peakV;//最大电压
  unsigned long peakVTime;//传感器最大波峰时间(ms)
  unsigned long orgPeakVTime;//原始、未经调整的传感器最大波峰时间
  int realOrder;//实际顺序，两个传感器序号可能相同
};
Sensor sortedSensor[SENSOR_COUNT];
Sensor sensor[SENSOR_COUNT];

void initVArray() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensor[i].curV = 0;
    sensor[i].avgV = 0;
    sensor[i].peakV = 0;
    sensor[i].peakVTime = 0;
    sensor[i].orgPeakVTime = 0;
    sensor[i].realOrder = 0;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Started!");
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
    if(sensor[s].peakV>0&&t-sensor[s].peakVTime>PEAK_ACTION_PERIOD){
       sensor[s].peakV = 0;
       sensor[s].peakVTime = 0;
    }
    //读取数据
    sensor[s].curV = analogRead(INPUT_PIN[s]);
    if(sensor[s].avgV<0.1){
      sensor[s].avgV=sensor[s].curV;
    }
    else{
      sensor[s].avgV = (sensor[s].avgV * (AVG_V_SAMPLE_COUNT - 1) + sensor[s].curV) / AVG_V_SAMPLE_COUNT;
    }
    //电压超过平均值的y%，才设置波峰
    if (sensor[s].curV > sensor[s].peakV
    && isAboveAvgV(sensor[s])) {
      sensor[s].peakV = sensor[s].curV;
      sensor[s].peakVTime = t;
      sensor[s].orgPeakVTime = t;
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
    +(sensors[i].peakVTime/1000)+(".")+(sensors[i].peakVTime % 1000)+"   orgPeakVTime:"+(sensors[i].orgPeakVTime/1000)+(".")+(sensors[i].orgPeakVTime % 1000)
    +"   realOrder:"+sensors[i].realOrder);
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
    if (isAboveAvgV(sensor[s])) {
      hasPeak=1;
      break;
    }
  }
  if(hasPeak==1){
    return;
  }
  
  //至少有一个Sensor有peak值
  for (int s = 0; s < SENSOR_COUNT; s++) {
    if (sensor[s].peakV >0.1) {
      return 1;
    }
  }
  //没有找到超出平均值的peakV
  return 0;
}

/**
   将识别出来的手势动作转换为红外信号发射
*/
void sendAction(int action) {
  Serial.print("Action:");Serial.println(action);
}
/**
   使用冒泡排序算法，根据波峰时间进行排序，排序结果放在sortedSensor数组中
*/
/*
void printSortResult(String sortType){
  Serial.println(String("***********  ")+sortType+"  ***********");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int sensorId=sortedSensor[i].sensorId;
    Serial.println(SENSOR_NAMES[sensorId]+"   sensorId:"+sensorId+"   peakV:"+sortedSensor[i].peakV+"   avgV:"+sortedSensor[i].avgV+"   peakVTime:"
    +(sortedSensor[i].peakVTime/1000)+(".")+(sortedSensor[i].peakVTime % 1000)+"   orgPeakVTime:"+(sortedSensor[i].orgPeakVTime/1000)+(".")+(sortedSensor[i].orgPeakVTime % 1000)+"   realOrder:"+sortedSensor[i].realOrder);
  }

}
*/
int recognizeAction_sort() {
  //先复制peakVTime数据到临时数组
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sortedSensor[i]=sensor[i];
  }
  
  //冒泡排序
  for (int i = 0; i < SENSOR_COUNT - 1; i++) //n个数的数列总共扫描n-1次
  {
    for (int j = 0; j < SENSOR_COUNT - i - 1; j++) //每一趟扫描到a[n-i-2]与a[n-i-1]比较为止结束
    {
      if (sortedSensor[j].peakVTime > sortedSensor[j + 1].peakVTime) //如果后一位数比前一位数小的话，就交换两个数的位置（升序）
      {
        Sensor t = sortedSensor[j + 1];
        sortedSensor[j + 1] = sortedSensor[j];
        sortedSensor[j] = t;
      }
    }
  }
  //根据排好序的波峰时间，给每个SENSOR指定实际序号
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sortedSensor[i].realOrder = i;
  }
}
/**
 * 对排序结果进行优化，如果两个时间接近，则调整为相同的时间点和顺序，允许一定时间点的误差
 */
void recognizeAction_refineOrder(){
   //获取开始/结束时间
  unsigned long firstTime=0;
  unsigned long lastTime = sortedSensor[SENSOR_COUNT - 1].peakVTime;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sortedSensor[i].peakVTime > 0) {
      //第一个不为0的时间为firstTime
      firstTime = sortedSensor[i].peakVTime;
      break;
    }
  }
  //调整排序结果
  long minSensorPeriodTime = (lastTime - firstTime) * MIN_SENSOR_PERIOD_RATE;
  Serial.print("firstTime:");Serial.print(firstTime);Serial.print(",lastTime:");Serial.print(lastTime);
  Serial.print(",MIN_SENSOR_PERIOD_RATE:");Serial.print(MIN_SENSOR_PERIOD_RATE);  Serial.print("，minSensorPeriodTime:"); Serial.println(minSensorPeriodTime);

  for (int i = 1; i < SENSOR_COUNT; i++) {
    //每个传感器与排在前面的传感器比较
    for (int j = 0; j < i; j++) {
      if (sortedSensor[i].peakVTime - sortedSensor[j].peakVTime <= minSensorPeriodTime) {
        //与前一个传感器差别小于指定参数，则调整时间点和顺序
        sortedSensor[i].peakVTime = sortedSensor[j].peakVTime;
        sortedSensor[i].realOrder = sortedSensor[j].realOrder;
        break;
      }
    }
  }
}
//将排序结果转换为整数类型的action，
int recognizeAction_covertToActionCode(){
  int sensorOrder[SENSOR_COUNT];
  for(int i=0;i<SENSOR_COUNT;i++){
    int sensorId=sortedSensor[i].sensorId;
    sensorOrder[sensorId]=sortedSensor[i].realOrder;
  }
  int actionCode=0;
  for(int i=0;i<SENSOR_COUNT;i++){
    int sensorId=sortedSensor[i].sensorId;
    actionCode=actionCode*10+sensorOrder[i];
  }
  return actionCode;
}
/**
   根据SensorData识别Action
*/
int recognizeAction() {
  recognizeAction_sort();
  printSensorData(1,"Sort Result",sortedSensor);
  recognizeAction_refineOrder();
  printSensorData(1,"Sort Refined Result",sortedSensor);
 resetSensorData();
  return recognizeAction_covertToActionCode();
}

void resetSensorData() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensor[i].curV = 0;
    sensor[i].peakV = 0;
    sensor[i].peakVTime = 0;
    sensor[i].orgPeakVTime = 0;
    sensor[i].realOrder = 0;
  }
}

void loop() {
  readSensorData();
  printSensorData(1,"",sensor);
  if (hasCompletedAction()) {
    Serial.println("New Action");
    printSensorData(0,"",sensor);
    int action = recognizeAction();
    sendAction(action);
    resetSensorData();
  }
  unsigned long t = millis();
  if(t-lastDebugTime>2000){
    Serial.println("No Action.");
    lastDebugTime=t;
  }
  //delay(LOOP_DELAY);
    delay(1000);
}

