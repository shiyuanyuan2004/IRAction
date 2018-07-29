#include "Arduino.h"

//红外传感器数目，2*2
const int SENSOR_COUNT = 4;

//传感器输入pin，数组下标按照从左到右、从上到下方式排列
const int INPUT_PIN[SENSOR_COUNT] = {A1, A2, A3, A4};
const String SENSOR_NAMES[SENSOR_COUNT] = {"S_1_1", "S_1_2", "S_2_1", "S_2_2"};
//每次循环延迟时间(ms)，要求小于5ms，太短可能提高cpu使用率
const int LOOP_DELAY = 5;

//计算平均电压的时间区间，转换为sample次数,设置为100秒，该值只影响取平均值的平滑度
const long AVG_V_SAMPLE_COUNT = 100L * 1000 / LOOP_DELAY;
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

////设定手势在1s内完成，超过1s重新计数
//const int totalPeriodInMs=1000;
////每次采样的时间区间,设定10ms，为了避免某个时间点被跳过，10ms内采样多次，取最后一次。
//const int perSamplePeriodInMs=10;
////采样区间数
//const int samplePeriodCount=totalPeriodInMs/perSamplePeriodInMs;

//各个传感器电压值，第一个维度是传感器，第二个是取样区间，循环记录在长度100的数组中。
//int v[SENSOR_COUNT][samplePeriodCount];

//判断波峰的最小值，相对于平均值
const float MIN_MAX_V_RATE = 1.2f;

//判断两个传感器先后顺序的最小间隔时间比例，以整个手势时间为基准。用于容错处理，允许少于的方向偏差
const float MIN_SENSOR_PERIOD_RATE = 0.1f;

//各个传感器的当前电压
int curV[SENSOR_COUNT];

//各个传感器的平均电压
int avgV[SENSOR_COUNT];
//各个传感器的最大电压
int maxV[SENSOR_COUNT];
//各个传感器的出现最大电压时的时间(ms)
long maxVTime[SENSOR_COUNT];

//排序用时间数组
struct TimeInfo {
  int sensorId;//传感器对应下标
  int maxVTime;//传感器最大波峰时间
  int orgMaxVTime;//原始、未经调整的传感器最大波峰时间
  int realOrder;//实际顺序，两个传感器序号可能相同
};
TimeInfo times[SENSOR_COUNT];

void initVArray() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    //    for(int j=0;j<samplePeriodCount;j++){
    //      v[i][j]=0;
    //    }
    curV[i] = 0;
    avgV[i] = 0;
    maxV[i] = 0;
    maxVTime[i] = 0;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
////所有取样数据往前移动一位
//void shiftSensorData(){
//  for(int sensor=0;sensor<SENSOR_COUNT;sensor++){
//    for(int pos=1;pos<samplePeriodCount;pos++){
//      v[s][pos-1]=v[s][pos];
//    }
//  }
//}
//读取所有传感器的电压，放在最后一个位置
void readSensorData() {
  long t = millis();
  //  shiftSensorData();
  for (int s = 0; s < SENSOR_COUNT; s++) {
    curV[s] = analogRead(INPUT_PIN[s]);
    avgV[s] = (avgV[s] * (AVG_V_SAMPLE_COUNT - 1) + curV[s]) / AVG_V_SAMPLE_COUNT;
    if (curV[s] > maxV[s]) {
      maxV[s] = curV[s];
      maxVTime[s] = t;
    }
  }
}

void printSensorData() {
  long t = millis();
  Serial.print("Time:");
  Serial.print(t / 1000);
  Serial.print(".");
  Serial.println(t % 1000);
  for (int s = 0; s < SENSOR_COUNT; s++) {
    Serial.print(SENSOR_NAMES[s]);
    Serial.print("   cur:");
    Serial.print(curV[s]);
    Serial.print("   avg:");
    Serial.print(avgV[s]);
    Serial.print("   max:");
    Serial.print(maxV[s]);
    Serial.print("   maxTime:");
    Serial.println(maxVTime[s]);
  }
}

/**
   判断是否有完成的手势，判断条件：
   1：所有Sensor的当前电压不超过平均值的x%
   2: 至少有一个Sensor有1个电压超过平均值的y%
*/
int hasCompletedAction() {
  //检查所有Sensor的当前电压不超过平均值的x%
  float maxCurVRate = 1.1f;
  for (int s = 0; s < SENSOR_COUNT; s++) {
    if (curV[s] > avgV[s]*maxCurVRate) {
      return 0;
    }
  }
  //至少有一个Sensor有1个电压超过平均值的y%
  for (int s = 0; s < SENSOR_COUNT; s++) {
    if (maxV[s] > avgV[s]*MIN_MAX_V_RATE) {
      return 1;
    }
  }
  //没有找到超出平均值的maxV
  return 0;
}

/**
   将识别出来的手势动作转换为红外信号发射
*/
void sendAction(int action) {
  Serial.print("Action:")
  Serial.println(action);
}
/**
   使用冒泡排序算法，根据波峰时间进行排序，排序结果放在times数组中
*/
int recognizeAction_sort() {
  //先复制maxVTime数据到临时数组
  long i, j;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    times[i].sensorId = i;
    times[i].maxVTime = maxVTime[i];
    times[i].orgMaxVTime = maxVTime[i];
  }
  //冒泡排序
  for (i = 0; i < SENSOR_COUNT - 1; i++) //n个数的数列总共扫描n-1次
  {
    for (j = 0; j < SENSOR_COUNT - i - 1; j++) //每一趟扫描到a[n-i-2]与a[n-i-1]比较为止结束
    {
      if (times[j].maxVTime > times[j + 1].maxVTime) //如果后一位数比前一位数小的话，就交换两个数的位置（升序）
      {
        TimeInfo t = times[j + 1];
        times[j + 1] = times[j];
        times[j] = t;
      }
    }
  }
  //根据排好序的波峰时间，给每个SENSOR指定实际序号
  for (int i = 0; i < SENSOR_COUNT; i++) {
    times[i].realOrder = i;
  }
}
/**
 * 对排序结果进行优化，如果两个时间接近，则调整为相同的时间点和顺序，允许一定时间点的误差
 */
void recognizeAction_refineOrder(){
   //获取开始/结束时间
  long firstTime;
  long lastTime = times[SENSOR_COUNT - 1].maxVTime;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (times[i].maxVTime > 0) {
      //第一个不为0的时间为firstTime
      firstTime = times[i].maxVTime;
      break;
    }
  }
  //调整排序结果
  long minSensorPeriodTime = (lastTime - firstTime) * MIN_SENSOR_PERIOD_RATE;
  for (int i = 1; i < SENSOR_COUNT; i++) {
    //每个传感器与排在前面的传感器比较
    for (int j = 0; j < i; j++) {
      if (times[i].maxVTime - times[j].maxVTime < minSensorPeriodTime) {
        //与前一个传感器差别小于指定参数，则调整时间点和顺序
        times[i].maxVTime = times[j].maxVTime;
        times[i].realOrder = times[j].realOrder;
        break;
      }
    }
  }
}
//将排序结果转换为整数类型的action，
int recognizeAction_covertToActionCode(){
  int sensorOrder[SENSOR_COUNT];
  for(int i=0;i<SENSOR_COUNT;i++){
    int sensorId=times[i].sensorId;
    sensorOrder[sensorId]=times[i].realOrder;
  }
  int actionCode=0;
  for(int i=0;i<SENSOR_COUNT;i++){
    int sensorId=times[i].sensorId;
    actionCode=actionCode*10+sensorOrder[i];
  }
  return actionCode;
}
/**
   根据SensorData识别Action
*/
int recognizeAction() {
  recognizeAction_sort();
  recognizeAction_refineOrder();
  return recognizeAction_covertToActionCode();
}

void resetSensorData() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    maxV[i] = 0;
    maxVTime[i] = 0;
  }
}

void loop() {
  readSensorData();
  printSensorData();
  if (hasCompletedAction) {
    int action = recognizeAction();
    sendAction(action);
    resetSensorData();
  }
  delay(LOOP_DELAY);
}

