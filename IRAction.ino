const int inputPin11=A1;
const int inputPin12=A2;
const int inputPin22=A3;
const int inputPin21=A4;
int delay0=0;
long count=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  count++;
  int v11=analogRead(inputPin11);
  delay(delay0);
  int v12=analogRead(inputPin12);
  delay(delay0);
  int v22=analogRead(inputPin22);
  delay(delay0);
  int v21=analogRead(inputPin21);
  delay(delay0);
  Serial.print("****************   ");
  Serial.print(count);
  Serial.print("   ****************");
  Serial.print(v11);
  Serial.print(',');
  Serial.println(v12);
  Serial.print(v21);
  Serial.print(',');
  Serial.println(v22);
  delay(500);
}

