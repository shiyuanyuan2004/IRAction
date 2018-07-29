#include "Arduino.h"

//红外传感器数目，2*2
const int sensorCount=4;

//传感器输入pin，数组下标按照从左到右、从上到下方式排列
const int inputPins[sensorCount]={A1,A2,A3,A4};

//每次循环延迟时间(ms)，要求小于5ms，太短可能提高cpu使用率
const int loopDelay=10;

//不同传感器之间取样间隔
const int sampleDelay=0;

//设定手势在1s内完成，超过1s重新计数
const int totalPeriodInMs=1000;
//每次采样的时间区间,设定10ms，为了避免某个时间点被跳过，10ms内采样多次，取最后一次。
const int perSamplePeriodInMs=10;
//采样区间数
const int samplePeriodCount=totalPeriodInMs/perSamplePeriodInMs;

//各个传感器电压值，第一个维度是传感器，第二个是取样区间，循环记录在长度100的数组中。
int v[sensorCount][samplePeriodCount];
//各个传感器的平均电压
int avgV[sensorCount];

void initVArray(){
  for(int i=0;i<sensorCount;i++){
    for(int j=0;j<samplePeriodCount;j++){
      v[i][j]=0;
    }
    avgV[i]=0;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
//所有取样数据往前移动一位
void shiftSensorData(){
  for(int sensor=0;sensor<sensorCount;sensor++){
    for(int pos=1;pos<samplePeriodCount;pos++){
      v[sensor][pos-1]=v[sensor][pos];
    }
  }
}
//读取所有传感器的电压，放在最后一个位置
void readSensorData(){
  shiftSensorData();
  for(int sensor=0;sensor<sensorCount;sensor++){
    v[sensor][samplePeriodCount]=analogRead(inputPins[sensor]);
    avgV[sensor]=(avgV[sensor]*(samplePeriodCount-1)+v[sensor])/samplePeriodCount;
  }
}

void printSensorData(){
	long t=millis();
	Serial.print("Time:");
	Serial.print(t/1000);
	Serial.print(".");
	Serial.println(t%1000);
	for(int sensor=0;sensor<sensorCount;sensor++){
		for(int pos=0;pos<samplePeriodCount;pos++){
			Serial.print(v[sensor][pos]);
			Serial.print("\t");
		}
		Serial.println();
    }
}

/**
 * 判断是否有完成的手势，判断条件：
 * 1：所有Sensor的当前电压不超过平均值的x%
 * 2: 至少有一个Sensor有连续n个电压超过平均值的y%
 */
int hasCompletedAction(){
	//检查所有Sensor的当前电压不超过平均值的x%
	float actionCompletedVCheckLevel=1.1f;
	for(int sensor=0;sensor<sensorCount;sensor++){
		if(v[sensor][samplePeriodCount]> avgV[sensor]*actionCompletedVCheckLevel){
			return 0;
		}
	}
	//至少有一个Sensor有连续n个电压超过平均值的y%
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
	  for(int i=0;i<sensorCount;i++){
	    for(int j=0;j<samplePeriodCount;j++){
	      v[i][j]=avgV[i];
	    }
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
  delay(loopDelay);
}

