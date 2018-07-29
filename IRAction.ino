#include "Arduino.h"

//红外传感器数目，2*2
const int SENSOR_COUNT=4;

//传感器输入pin，数组下标按照从左到右、从上到下方式排列
const int INPUT_PIN[SENSOR_COUNT]={A1,A2,A3,A4};
const String[] SENSOR_NAMES={"S_1_1","S_1_2","S_2_1","S_2_2"};
//每次循环延迟时间(ms)，要求小于5ms，太短可能提高cpu使用率
const int LOOP_DELAY=5;

//计算平均电压的时间区间，转换为sample次数,设置为100秒，该值只影响取平均值的平滑度
const long AVG_V_SAMPLE_COUNT=100L*1000/LOOP_DELAY;
//不同传感器之间取样间隔
const int SAMPLE_DELAY=0;

const int ACTION_LEFT=1;
const int ACTION_RIGHT=2;
const int ACTION_UP=3;
const int ACTION_DOWN=4;
const int ACTION_LEFT_UP=5;
const int ACTION_LEFT_DOWN=6;
const int ACTION_RIGHT_UP=7;
const int ACTION_RIGHT_DOWN=8;

////设定手势在1s内完成，超过1s重新计数
//const int totalPeriodInMs=1000;
////每次采样的时间区间,设定10ms，为了避免某个时间点被跳过，10ms内采样多次，取最后一次。
//const int perSamplePeriodInMs=10;
////采样区间数
//const int samplePeriodCount=totalPeriodInMs/perSamplePeriodInMs;

//各个传感器电压值，第一个维度是传感器，第二个是取样区间，循环记录在长度100的数组中。
//int v[SENSOR_COUNT][samplePeriodCount];

//各个传感器的当前电压
int curV[SENSOR_COUNT];

//各个传感器的平均电压
int avgV[SENSOR_COUNT];
//各个传感器的最大电压
int maxV[SENSOR_COUNT];
//各个传感器的出现最大电压时的时间(ms)
long maxVTime[SENSOR_COUNT];

void initVArray(){
  for(int i=0;i<SENSOR_COUNT;i++){
//    for(int j=0;j<samplePeriodCount;j++){
//      v[i][j]=0;
//    }
    curV[i]=0;
    avgV[i]=0;
    maxV[i]=0;
    maxVTime[i]=0;
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
void readSensorData(){
	long t=millis();
//  shiftSensorData();
  for(int s=0;s<SENSOR_COUNT;s++){
	curV[s]= analogRead(INPUT_PIN[s]);
    avgV[s]=(avgV[s]*(AVG_V_SAMPLE_COUNT-1)+curV[s])/AVG_V_SAMPLE_COUNT;
    if(curV[s]>maxV[s]){
    	maxV[s]=curV[s];
    	maxVTime[s]=t;
    }
  }
}

void printSensorData(){
	long t=millis();
	Serial.print("Time:");
	Serial.print(t/1000);
	Serial.print(".");
	Serial.println(t%1000);
	for(int s=0;s<SENSOR_COUNT;s++){
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
 * 判断是否有完成的手势，判断条件：
 * 1：所有Sensor的当前电压不超过平均值的x%
 * 2: 至少有一个Sensor有1个电压超过平均值的y%
 */
int hasCompletedAction(){
	//检查所有Sensor的当前电压不超过平均值的x%
	float maxCurVRate=1.1f;
	for(int s=0;s<SENSOR_COUNT;s++){
		if(curV[s]> avgV[s]*maxCurVRate){
			return 0;
		}
	}
	//至少有一个Sensor有连续n个电压超过平均值的y%
	float minMaxVRate=1.2f;
	for(int s=0;s<SENSOR_COUNT;s++){
		if(maxV[s]> avgV[s]*minMaxVRate){
			return 1;
		}
	}
	//没有找到超出平均值的maxV
	return 0;
}

/**
 * 将识别出来的手势动作转换为红外信号发射
 */
void sendAction(int action){

}

/**
 * 根据SensorData识别Action
 */
int recognizeAction(){

}

void resetSensorData(){
  for(int i=0;i<SENSOR_COUNT;i++){
	maxV[i]=0;
	maxVTime[i]=0;
  }
}

void loop() {
  readSensorData();
  printSensorData();
  if(hasCompletedAction){
	  int action=recognizeAction();
	  sendAction(action);
	  resetSensorData();
  }
  delay(LOOP_DELAY);
}

