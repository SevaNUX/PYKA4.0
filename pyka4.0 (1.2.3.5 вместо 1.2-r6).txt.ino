/*
 Модель робота: PYKA 4.01
 Контроллер: Arduino nano
 */

// --------- Библиотеки ---------
#include <Servo.h>  //библиотека серво
#include <Adafruit_VL53L0X.h>  //библиотека сенсора
#include <IRremote.h>

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();  //инициируем дальномер как сеснор

// --------- Настройки ---------
#define STEP_DELAY 15		// Переменная для установки пауз
#define MAXDIST 380		// Максимальное расстояние (см). Датчик бьёт до 4 метров, но будут шумы
#define TARGETZONE 50     // gjhjujdjt pyfxtybt
#define SRV_AMOUNT 7		// Приводы  
#define LASER A0 // нижний лазер
#define LASER1 A1 // верхний лазер

const int RECV_PIN = 13; // Пин для ИК приёмника
IRrecv irrecv(RECV_PIN);
decode_results results;

byte SRV_PIN[SRV_AMOUNT] = {6, 7, 8, 9, 10, 11, 12};               // Пины серв
byte SRV_DEF[SRV_AMOUNT] = {90, 85, 90, 99, 100, 90, 40};          // Положение по умолчанию
byte SRV_CUR[SRV_AMOUNT] = {90, 85, 90, 99, 100, 90, 40};         // Текущее положение сер
byte SRV_MIN[SRV_AMOUNT] = {0, 26, 0, 0, 0, 0, 40};                // Минимальная позиция поворота
byte SRV_MID[SRV_AMOUNT] = {90, 90,  90, 90, 90, 84};  // Среднее позиция поворота
byte SRV_MAX[SRV_AMOUNT] = {180, 270, 180, 180, 180, 180, 114};  // Максимальная позиция поворота

int TARGET[181]; // Массив радара 
int dist; // расстояние для датчика растояния
int TARG_IN=0;  // начало цели
int TARG_OUT=0;  //конец цели
float Xp, Yp, Zp, Qp;// Переменные для хранения введённых значений
String inputStrX, inputStrY, inputStrZ; // Строки для временного хранения ввода
float tempValue;    
int a0, a1, a2, a3;
float targetX = 0.0;
float targetY = 0.0;
float targetZ = 0.0;

// Время отклика для каждой команды
unsigned long lastCommandTime = 0;
unsigned long DELAY_BETWEEN_COMMANDS = 200;
bool inverseKinematics(float x, float y, float z) {
  double a = sqrt(x * x + y * y); // Расстояние до основания по горизонтали
  double b = z;                   // Высота вертикальная
  double c = pow(L1, 2) + pow(L2, 2) + pow(L3, 2) + pow(L4, 2) - pow(a, 2) - pow(b, 2);

  // Достижимость точки
  if(c > 0 && c <= 4*(L1*L2 + L3*L4)){
    double phi = atan2(y, x);              // Ориентация базы относительно горизонта
    double theta1 = acos((pow(L1, 2) + pow(L2, 2) - pow(a, 2))/(-2*L1*L2));  
    double theta2 = asin(z/(sqrt(pow(L1, 2)+pow(L2, 2)))) - theta1;          
    double theta3 = atan2(z-a*sin(theta2),a*cos(theta2)-b);                   
    
    // Перевод углов в диапазон серво (от 0 до 180 градусов)
    angleBase = radiansToDegrees(phi);
    angleShoulder = radiansToDegrees(theta1);
    angleElbow = radiansToDegrees(theta2);
    angleWrist = radiansToDegrees(theta3);

    return true;
  }
  else{
    return false; // Точка вне зоны досягаемости
  }
}

// Преобразование радиан в градусы
double radiansToDegrees(double rad){
  return rad * 180.0 / PI;
}



// Принятие данных через COM-порт
void serialInput(){
  String inputString = "";
  char inChar;
  bool stringComplete = false;

  while(Serial.available()){
    inChar = Serial.read();
    if(inChar != '\n'){
      inputString += inChar;
    }
    else{
      stringComplete = true;
    }
  }

  if(stringComplete){
    // Парсим строку вида "X Y Z"
    sscanf(inputString.c_str(), "%f %f %f", &targetX, &targetY, &targetZ);
    Serial.print("Received coordinates: ");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.print(targetY);
    Serial.print(", ");
    Serial.println(targetZ);
  }
}

byte SRV = 0;			//Дефолтное значение сервы
byte VALUE = 0;			//Дефолтное значение угла
byte scanmode = true; // режим сканирования
// boolean catch_flag, catched_flag, hold_flag;
Servo SDRV[SRV_AMOUNT]; // Инициализация сервомоторов

// Инициализация, задание начальных установок.
// Выполняется один раз при старте.

void setup() {
// --------- Консоль ---------
  Serial.begin(9600); 
  while (! Serial) {
    delay(1);
  }
  Serial.println("---- Go ----");
// Инициализация основных контактов сервомашинок и 
  for (int i = 0; i < 7; i++) {
	  SDRV[i].attach(SRV_PIN[i]);		// Инициализация пинов сервомашинок
	  SDRV[i].write(SRV_DEF[i]);		// Установка начального положения
	  SRV_CUR[i]=SRV_DEF[i];		// Установка текущего положения
    //Serial.print("Drive - ");Serial.print(i);
  }  

// --------- Сенсор ---------
  Serial.println("VL53L0X test");
  if (!sensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
   // while(1);
  }
  pinMode(LASER, OUTPUT); // инициализируем Pin10 как выход
  digitalWrite(LASER, LOW);
 // TARGET_SCAN();  // забиваем калибровочный массив
}

void loop() {
CALIBRATION();
/if (scanmode) SCAN();       // режим поиска цели
  //else delay(1);    
  /*
  if (scanmode) {
  r=68;
  a0=59;
  MATH();
  }     // режим поиска цели
   else delay(1);  
  */  if(irrecv.decode(&results)) {
    unsigned long currentMillis = millis();
    if(currentMillis - lastCommandTime >= DELAY_BETWEEN_COMMANDS){      
      switch(results.value){
        case 0xFFE21D: 
          targetY++;
          break;
        case 0xFFF20D: 
          targetY--;
          break;
        case 0xFFA857: 
          targetX--;
          break;
        case 0xFFB04F: 
          targetX++;
          break;
        default:
          break;
      }
      lastCommandTime = currentMillis;
    }
    irrecv.resume();                  
  }

  // Чтение команд через COM-порт
  serialInput();

  if(inverseKinematics(targetX, targetY, targetZ)){
   MOVETO();
  }

  delay(100);                       
}



}


void MOVETO(){   // Крутим сервой
int angle;
if (SRV_CUR[SRV]<=VALUE) {  // Едем от текущего угла к целевому
  for (angle = SRV_CUR[SRV]; angle <= VALUE; angle++) {
    SDRV[SRV].write(angle);
    delay(STEP_DELAY * 1.5*2);
  }
}
  else {
  for (angle = SRV_CUR[SRV]; angle >= VALUE; angle--) { // Едем от текущего угла к целевому
    SDRV[SRV].write(angle);
    delay(STEP_DELAY * 1.5*2);
    }
  }
SRV_CUR[SRV]=VALUE;
}

void CALIBRATION() {
/*SRV=6;
VALUE=SRV_MIN[SRV]; MOVETO();
delay(STEP_DELAY * 1.5);
for (int angle = 26; angle <= 270 ; angle++) { // SRV_MIN[SRV] SRV_MAX[SRV] 
    SDRV[SRV].write(angle);
    if (angle==SRV_MID[SRV]){delay(STEP_DELAY * 1.5);}
    delay(STEP_DELAY * 1.5);
    delay(STEP_DELAY * 70);
    Serial.print("DRV "); Serial.print(SRV); Serial.print(" = "); Serial.println(angle);
   SRV_CUR[SRV]=angle;
   }
   delay(STEP_DELAY * 1.5);
VALUE=SRV_MIN[SRV]; MOVETO();*/
for (int angle = 0; angle <= SRV_AMOUNT ; angle++)
{
 SDRV[angle].write(SRV_DEF[angle]);
Serial.print("DRV ");
Serial.print(angle);
Serial.println("parking done");
delay(STEP_DELAY * 1.5*2);
}

//Serial.println("Servo calibration done");
}

void TARGET_SCAN() {  // ЗАПОЛНЯЕМ МАССИВ РАССТОЯНИЙ
  digitalWrite(LASER, HIGH);
  SRV=0; VALUE=SRV_MIN[SRV]; MOVETO();
  delay(STEP_DELAY * 1.5);
  for (int val = SRV_MIN[SRV]; val <= SRV_MAX[SRV]; val++) {
    VALUE=val; MOVETO();
      Measure();
    if ( dist>= 110 && dist <=MAXDIST){
      TARGET[val]=dist;
      delay(STEP_DELAY * 1.5);
     }
    // Serial.print("Angle "); Serial.print(mov); Serial.print(" Array "); Serial.println(r);
    SRV_CUR[SRV]=val;
    delay(STEP_DELAY * 1.5);
  }

  SRV=0; VALUE=SRV_MIN[SRV]; MOVETO();
  Serial.println("Radar init done");
  digitalWrite(LASER, LOW);
}

void SCAN() {  // сравниваем с массивом рассточяний
  digitalWrite(LASER, HIGH);
  SRV=0;
  for (int val = SRV_MIN[SRV]; val <= SRV_MAX[SRV]; val++) {
    VALUE=val; MOVETO();
    SRV_CUR[SRV]=val;
      Measure();
    //Serial.print("Ang="); Serial.print(i); Serial.print(" Sens=");Serial.print(r); Serial.print(" Array=");Serial.print(TARGET[i]);Serial.print(" Delta=");Serial.println(TARGET[i]-r);
    if (TARG_IN==0 && (TARGET[val]-dist) >= TARGETZONE) {
      TARG_IN=val;
      //Serial.print("targin"); Serial.println(TARG_IN);
    }
    if (TARG_IN !=0 && (TARGET[val]-dist)>=TARGETZONE) {
    TARG_OUT=val;
    //Serial.print("targout"); Serial.println(TARG_OUT);
    }
    if (TARG_OUT != 0 && (TARGET[val]-dist)<TARGETZONE) {
      HOLDTARGET();
      //Serial.print("Target"); Serial.println(TARG_IN-TARG_OUT);
    }
    delay(STEP_DELAY * 1.5);
    if (scanmode==false) break;
    }

  for (int val = SRV_MAX[SRV]; val >= SRV_MIN[SRV]; val--) {
    VALUE=val; MOVETO();
    SRV_CUR[SRV]=val;
      Measure();
      Serial.print("Ang="); Serial.print(val); Serial.print(" Sens=");Serial.print(dist); Serial.print(" Array=");Serial.print(TARGET[val]);Serial.print(" Delta=");Serial.println(TARGET[val]-dist);
    if (TARG_IN==0 && (TARGET[val]-dist) >= TARGETZONE) {
    TARG_IN=val;
    //Serial.print("targin"); Serial.println(TARG_IN);
    }
    if (TARG_IN !=0 && (TARGET[val]-dist)>=TARGETZONE) {
    TARG_OUT=val;
    //Serial.print("targout"); Serial.println(TARG_OUT);
    }
    if ( TARG_OUT != 0 && (TARGET[val]-dist)< TARGETZONE) {
    HOLDTARGET();
    //Serial.print("Target"); Serial.println(TARG_IN-TARG_OUT);
    }
    delay(STEP_DELAY * 1.5);
    if (scanmode==false) break;
  }
    digitalWrite(LASER, LOW);
}


void MATH(){

// Данные с сенсора
Xpos=cos(a0/57.3)*(dist-0); //0 заменить на расстояниие от датчика до оси 0)
Ypos=sin(a0/57.3)*(dist-0); //0 заменить на расстояниие от датчика до оси 0)

int L0=100;
int L1=94;
int L2=61;
int L3=74;

//int Zpf=Zp+L0;

  

float Xt = sqrt(pow(Xp, 2) + pow(Yp, 2)); // Координата X целевой  точки в основной плоскости движения звеньев 60, 90 и 10
float dX = L3*sin(Qp/57.3); // Корректировка X на ориентацию 3-го звена (10)
float dY = L3*cos(Qp/57.3); // Корректировка Y на ориентацию 3-го звена (10)
float Xn = Xt - dX; // Позиция X в основной плоскости, куда должен попасть шарнир 3-го звена
float Yn = Zp + dY; // Позиция Y в основной плоскости, куда должен попасть шарнир 3-го звена
float d = sqrt(pow(Xn, 2) + pow(Yn, 2)); // Длина отрезка между 1-м и 3-м  шарнирами
float q1 = atan2(Yn, Xn)*57.3; // Вспомогательный угол 1
float q2 = acos((pow(d, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * d))*57.3;
float q3 = acos((pow(L1, 2)+ pow(L2, 2)  - pow(d, 2)) / (2 * L1 * L2))*57.3;

a0 = atan2(Yp, Xp)*57.3;
a1 = q1 + q2; // Угол поворота сервомотора 1-й оси относительно горизонтали
a2 = 180 - q3; // Угол поворота сервомотора 2-й оси относительно 1-й
a3 =  (180-q2 - q3)+(180-q1-90)-90+Qp; // Угол поворота 3-й оси относительно перпендикуляра к 2-й оси 
float result = Xp * Yp* Zp; 
Serial.print(Xp);Serial.print("/"); Serial.println(Yp);
scanmode=false;
}

//------------------- поймали цель -------------
void HOLDTARGET(){
  if (TARG_IN<TARG_OUT) {
    VALUE=TARG_IN+((TARG_OUT-TARG_IN)/2);
  }
  else
  {
    VALUE=TARG_OUT+((TARG_IN-TARG_OUT)/2);
  }
  a0=VALUE; SRV=0; MOVETO();
  Serial.print(TARG_IN);Serial.print("/");Serial.print(TARG_OUT);Serial.print("/");Serial.println(VALUE);
  for (int i=0; i<=24; i++) {
      digitalWrite(LASER, HIGH);
      delay(50);
      digitalWrite(LASER, LOW);
      delay(50);
   }
  delay(5000); 
  scanmode=false;
}

void Measure(){
long sum = 0; // локальная переменная sum
  for (int i = 0; i < 5; i++) {      // усредненяем
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false); // Заменить фолс на тру для дебага
    // if (measure.RangeStatus != 4){
    dist = measure.RangeMilliMeter;
    if (dist > MAXDIST) {
      dist = MAXDIST;
    }
      sum += dist; 
      Serial.print(dist); Serial.print(", ");
  }
    Serial.print(" sred "); Serial.print(sum/5);     Serial.println(" data ");                    // для примера выводим в порт  
  dist=sum/5;  
}
