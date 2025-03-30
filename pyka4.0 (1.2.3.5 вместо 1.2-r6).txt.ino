// ---------- БИБЛИОТЕКИ ----------
#include <Wire.h>                       // Подключаем Wire
//#include <Adafruit_PWMServoDriver.h>    // Подключаем Adafruit_PWMServoDriver
#include "GyverEncoder.h"               // Библиотека энкодера
#include "GyverHacks.h"
#include "LiquidCrystal_I2C.h"          // Библиотека экрана
#include "Servo.h"
#include <NewPing.h>
#include "Adafruit_VL53L0X.h" 
// ---------- БИБЛИОТЕКИ ----------

// ------------- НАСТРОЙКИ --------------
#define STEP_DELAY 15   
#define TIMEOUT 3000    // таймаут на новый поиск цели из режима удержания
#define MAX_ANGLE 140   // максимальный угол поворота
#define MIN_ANGLE 0    	// минимальный угол поворота
#define DIST_MAX 50    	// максимальное расстояние (см). 
#define DEADZONE 10     // зона нечувствительности (мин. разность с калибровкой)
#define MIN_CATCH 3    // мин. количество точек подряд, чтобы считать цель целью
#define MISTAKES 4      // допустимое количество пропусков при сканировании цели
#define SERVOMIN  50     // Минимальная длительность импульса для сервопривод
#define SERVOMAX  600     // Максимальная длина импульса для сервопривода
#define SrvAmount 7      //Количество приводов

char* ModeName[7] = {"Calibrate", "Auto", "Demo1", "Demo2","Demo3","Manual", "System"}; //Режимы

// ------------- НАСТРОЙКИ --------------

// ---------- ПИНЫ ----------
#define ECHO A6 // проверь
#define TRIG A7 // проверь
//#define MOS 2
#define LASER A0 // задаем имя для A0
#define LA A2 // привод вперед   проверь
#define LB A3 // привод назад     проверь
// --- Порты энкодера --
#define CLK 3
#define DT 4
#define SW 5
// --- Порты энкодера --
int SrvPins[7] = {6, 7, 8, 9, 10, 11, 12}; 
// ---------- ПИНЫ ----------

// ---------- ДЛЯ СКАНЕРА ----------
GTimer stepTimer(STEP_DELAY);
GTimer sonarTimer(100);
GTimer timeoutTimer(TIMEOUT);
NewPing sonar(TRIG, ECHO, DIST_MAX);
boolean direct;
boolean next;
const byte steps_num = (MAX_ANGLE - MIN_ANGLE) / 2;
int angle = MIN_ANGLE;
int distance[steps_num + 1];
boolean catch_flag, catched_flag, hold_flag;
byte catch_num;
byte mistakes;
byte mode = true;
byte catch_pos;
int hold_signal;

int ModeCur=8; //Текущий режим
int Mode = 0;
int ModeAmount=7;  //Количество режимов
int Srv = 0; //Дефолтное значение
int value = 0; //Дефолтное значение
int ServoCur=0;           // Текущий серво 
int i;

int DefAngle[7] = {90, 90, 27, 90, 42, 40, 90};  //Положение по умолчанию
int CurAngle[7] = {90, 90, 27, 90, 42, 40, 90};  //Текущее положение
byte mode2prg[50][2]; //Массив режима обучения


byte Prg01[5][7] = {
  {90, 90, 90, 36, 42, 40, 90},
  {90, 107, 90, 36, 42, 40, 90},
  {90, 107, 114, 36, 42, 40, 90},
  {90, 107, 90, 8, 42, 40, 90},
  {90, 90, 27, 90, 42, 40, 90},
};

byte prg1[6][2]= {
{2,107},
{3,180},
{2,175},
{1,98},
{0,149},
{2,170},
};

byte prg11[8][2]= {
{3,36},
{2,31},
{1,109},
{2,85},
{1,66},
{3,2},
{4,151},
{3,7},
};

byte prg2[7][2]= {
{0,126},
{2,152},
{1,15},
{3,0},
{2,63},
{0,9},
{1,97},
};

byte prg21[3][2]= {
{5,126},
{5,43},
{4,152},
};

byte prg3[12][2]= {
{1,151},
{2,149},
{1,148},
{3,136},
{4,144},
{5,170},
{1,160},
{1,116},
{0,49},
{2,172},
{3,180},
{5,124},
};
// ---------- ПЕРЕМЕННЫЕ ----------

LiquidCrystal_I2C lcd(0x27, 16, 2);
Encoder enc1(CLK, DT, SW);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Servo sdrive[SrvAmount]; //Приводы

void setup() {
    //pwm.begin();                // Инициализация
    // pwm.setPWMFreq(60);        // Частота следования импульсов 60 Гц
    // delay(10);                 // Пауза
    //  sdrive[0].attach(6); //Ось 0 - поворот вокруг Z
    //  sdrive[1].attach(7); //Ось 1 - Редуктор. наклон 0-180 (90 вперед)
    //  sdrive[2].attach(8); //Ось 2 - Наклон 0-180 (90 середина)
    //  sdrive[3].attach(9); //Ось 3 - Наклон рабочего органа
    //  sdrive[4].attach(10); // Схват
    //  sdrive[5].attach(11); // Сканер (резерв)
    //enc1.setTickMode(AUTO); //опросчик энкодера

    enc1.setType(TYPE2);  //тип энкодера 
    enc1.setFastTimeout(40);    // таймаут на скорость isFastR. По умолч. 50
    pinMode(LASER, OUTPUT); // инициализируем Pin10 как выход
  digitalWrite(LASER, LOW);
    Serial.begin(9600);             //Консоль
    Serial.println("Go-------->>>");
    Serial.println(ModeName[Mode]);
      Serial.println(sizeof(Prg01));
    Serial.println("--------------------------"); 
    Serial.print("Mode - ");
    Serial.println(ModeAmount);  
    pinMode(LA, OUTPUT);
    pinMode(LB, OUTPUT);
      digitalWrite(LA, LOW);
        digitalWrite(LB, LOW);
    pinMode(ECHO, OUTPUT);
    pinMode(TRIG, OUTPUT);

    Serial.println("Adafruit VL53L0X test");
    if (!lox.begin()) {
      Serial.println(F("Failed to boot VL53L0X"));
     while(1);
    }
 

    // Инициализация и начало работы с дисплеем
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0); // Устанавливаем положение курсора, 0-й, 0-я строка
    lcd.print("PYKA-4.0 by SEVA"); // Выводим текст
    lcd.setCursor(0, 1);
    lcd.print("Hi there!");
    
    initdrive();
    calibration(); 

    delay(3000);
  
    lcd.clear();  //Чистим экран
    lcd.setCursor(0, 0);
    lcd.print("Mode");
    lcd.setCursor(4, 0);
    lcd.print(">");
    lcd.setCursor(5, 0);
    lcd.print(ModeName[Mode]);
 }

void loop() {

 //HomePosition();
 enc1.tick(); // не нужна, в этом режиме (AUTO) она входит в каждую функцию
   
  if (enc1.isRight()) Mode--;         // если был поворот
  if (enc1.isLeft()) Mode++;
 
  if (enc1.isClick()) {
    ModeCur=Mode;
    lcd.setCursor(4, 0);
    lcd.print("-");
    lcd.setCursor(5, 0);
    lcd.print(ModeName[Mode]);
    MenuSwitch();
  }
 
     //  if (enc1.isHolded()) Serial.println("Holded");       // выход из режима

   if (enc1.isTurn()) {     // если был совершён поворот
     if(Mode>ModeAmount) Mode=0;
     if(Mode<0) Mode=7; 
    Serial.print("Mode - ");
    Serial.println(Mode);
     lcd.setCursor(5, 0);
     lcd.print("          ");
     lcd.setCursor(5, 0);
     lcd.print(ModeName[Mode]);
  }

}

void MenuSwitch()
{
  // свитч крутится в цикле loop
  switch (ModeCur) {
    case 0: task_0();
      break;
    case 1: task_1();
      break;
    case 2: task_2();
      break;
    case 3: task_3();
      break;
    case 4: task_4();
      break;
    case 5: task_5();
      break;
    case 6: task_6();
      break;
  }
}

// Задачи (режимы)
void task_0() { // Калибровка приводов для сборки манипулятора
  do {
enc1.tick();
 Serial.println("Калибровка приводов для сборки манипулятора");

 if (enc1.isRight()) value--;         // если был поворот
 if (enc1.isLeft()) value++;
 if (enc1.isFastR()) value -= 10;    // если был быстрый поворот направо, 
 if (enc1.isFastL()) value += 10;
      //  if (enc1.isHolded()) Serial.println("Holded");       // выход из режима
   if (enc1.isTurn()) {     // если был совершён поворот
     if(value>180) value=0;
      if(value<0) value=180; 
    Serial.print(value);
     // Serial.println(pulselen);
    lcd.setCursor(0, 1);
     lcd.print("                ");
     lcd.setCursor(0, 1);
     lcd.print(value);

  }
if (enc1.isClick()) sdrive[0].write(value);
if (enc1.isHolded()){ // Выход из этого меню
  ModeCur=8;       
  lcd.setCursor(4, 0);
  lcd.print(">");
 }

} while (ModeCur==0);
}


void task_1() {
  disp1();
  Serial.println("Режим АВТО");
  do {
    enc1.tick();
    if (mode) seek();       // режим поиска цели
      else hold();  
 if (enc1.isHolded()) ModeCur=8;   
  } while (ModeCur=1);
}
// ----------------------------------------------------------------------

void task_2() { // Программа 1
 disp1();
 Serial.println("Программа 1");
 do {
  enc1.tick();
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Drv-");
  lcd.setCursor(6, 1);
  lcd.print("Angle");
  for (int prgString = 0; prgString < 4; prgString++) {
    for (int prgElement = 0; prgElement < 6; prgElement++) {
      Srv=prgElement;
      value=Prg01[prgString][prgElement]; //1,2,3
      lcd.setCursor(4, 1);
      lcd.print("  ");
      lcd.setCursor(4, 1);
      lcd.print(Srv);
      lcd.setCursor(12, 1);
      lcd.print("   ");
      lcd.setCursor(12, 1);
      lcd.print(value);
      
      Serial.println("Drv - ");
      Serial.print(Srv);
      Serial.print(" / Angle - ");
      Serial.print(value);

      delay(500);
      RotateServo();
    }
    

  }
  wealding();
  parkdrive();
  ModeCur=8;
  
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Demo1 - Done");
  delay (3000);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  if (enc1.isHolded()){
    ModeCur=8;       // 8 режим главнго меню
    lcd.setCursor(4, 0);
    lcd.print(">");
  } 
  } while (ModeCur==2);
}


//-----------------------------------------------------------------------
void task_3() { //Пррограмма 2
 disp1();
 do {
  enc1.tick();


 for (int prg = 0; prg < ((sizeof(prg1)/2)); prg++) {
    
  //  arm[(prg1[prg][0])].write(prg1[prg][1]);
   
    Srv=prg1[prg][0];
    value=prg1[prg][1];
     Serial.println(prg1[prg][1]); 
    
    RotateServo();
    delay(500);
    }
    wealding();

    for (int prg = 0; prg < ((sizeof(prg11)/2)); prg++) {
 Srv=prg11[prg][0];
    value=prg11[prg][1];
     Serial.println(prg11[prg][1]); 
    RotateServo();
    delay(500);

    }




  wealding();

 if (enc1.isHolded()){
    ModeCur=8;       // 8 режим главнго меню
    lcd.setCursor(4, 0);
    lcd.print(">");
  } 
  } while (ModeCur==3);

}

//------------------------------------------------------------------------
void task_4() { // Программа 3
 disp1();
 do {
  enc1.tick();

for (int prg = 0; prg < ((sizeof(prg1)/2)); prg++) {
    
  //  arm[(prg1[prg][0])].write(prg1[prg][1]);
   
    Srv=prg2[prg][0];
    value=prg2[prg][1];
     Serial.println(prg2[prg][1]); 
    
    RotateServo();
    delay(500);
    }
    wealding();

    for (int prg = 0; prg < ((sizeof(prg11)/2)); prg++) {
 Srv=prg21[prg][0];
    value=prg21[prg][1];
     Serial.println(prg21[prg][1]); 
    RotateServo();
    delay(500);

    }
  drill();
  

    for (int prg = 0; prg < ((sizeof(prg3)/2)); prg++) {
 Srv=prg3[prg][0];
    value=prg3[prg][1];
     Serial.println(prg3[prg][1]); 
    RotateServo();
    delay(500);
    }



if (enc1.isHolded()){
    ModeCur=8;       // 8 режим главнго меню
    lcd.setCursor(4, 0);
    lcd.print(">");
  } 
  } while (ModeCur==4);
}


//-----------------------------------------------------------------------
void task_5() {
  Serial.println("Режим ручного управления");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Drv");
  do {
    enc1.tick();
    if (enc1.isRight()) Srv--;         // если был поворот
    if (enc1.isLeft()) Srv++;
    if (enc1.isTurn()) {     // если был совершён поворот
      if(Srv>SrvAmount) Srv=0;
      if(Srv<0) Srv=SrvAmount; 
      lcd.setCursor(4, 1);
      lcd.print(Srv);
    }
    if (enc1.isClick()) {
      ModeCur=51; 
      lcd.setCursor(6, 1);
      lcd.print("Angle");
      do {
        enc1.tick();
        if (enc1.isRight()) value--;         // если был поворот
        if (enc1.isLeft()) value++;
        if (enc1.isTurn()) {     // если был совершён поворот
        if(value>180) value=0;
        if(value<0) value=180; 
        lcd.setCursor(12, 1);
        lcd.print("   ");
        lcd.setCursor(12, 1);
        lcd.print(value);
      }
      if (enc1.isClick()) RotateServo();     // sdrive[Srv].write(value);
      if (enc1.isHolded()){ // Выход из этого меню
        ModeCur=5;       
        lcd.setCursor(6, 1);
        lcd.print("         ");
      }
      } while(ModeCur==51);
    }
    if (enc1.isHolded()){ // Выход из этого меню
      ModeCur=8;   
      parkdrive();    
      lcd.setCursor(4, 0);
      lcd.print(">");
      lcd.setCursor(0, 1);
      lcd.print("     ");
    }
  } while(ModeCur==5);
}

void task_6() {
  
}
 



// ---------- ФУНКЦИИ ---------------
 
void initdrive(){
    Serial.println("initdrive"); 
  //Включаем приводы
  for (i = 0; i < 7; i++) {
    value=DefAngle[i];
    sdrive[i].attach(SrvPins[i]);
    sdrive[i].write(value);
    CurAngle[i]=value;
    Serial.println("drv = "); 
Serial.print(i); 
 Serial.print(" angle = "); 
Serial.print(value); 
Serial.print(" Cur = "); 
Serial.print(CurAngle[i]); 

  }   
}

void parkdrive(){
  Serial.println("------------parking----------------");
  for (int pdrive = 0; pdrive < 7; pdrive++) {
    Srv=pdrive;
    value=DefAngle[pdrive];
   
    Serial.println(Srv);
Serial.println("Angle=");
Serial.print(value);
    RotateServo();

  } 
    Serial.println("------------parking done----------------");  
}

void unlinkdrive(){
  for (i = 0; i < 7; i++) {
    sdrive[i].detach();
  } 
}

void RotateServo(){
Serial.println("крутим серву");
/*Serial.println(Srv);
Serial.println("CurAngle=");
Serial.print(CurAngle[Srv]);
Serial.print(" / DestAngle=");
Serial.print(value);
*/
//sdrive[Srv].attach(SrvPins[Srv]);


if (CurAngle[Srv]<=value) {
//   Serial.println(" Mode i++ i=(");
 
  for (i = CurAngle[Srv]; i <= value; i++) {
   // Serial.print(i);
  //  Serial.print(", ");
  sdrive[Srv].write(i);
   delay(STEP_DELAY * 1.5);
}
 //Serial.print(")");

} else {
 //  Serial.println(" Mode i-- i=(");
 for (i = CurAngle[Srv]; i >= value; i--) {
  //Serial.print(i);
 //   Serial.print(", ");
  sdrive[Srv].write(i);
   delay(STEP_DELAY * 1.5);
}
//Serial.print(")");
}
CurAngle[Srv]=value;
Serial.println("крутим серву ----- готово");
}


void disp1() {  // Обновим дисплей (режим)
  lcd.setCursor(4, 0);
  lcd.print("-");
  lcd.setCursor(5, 0);
  lcd.print(ModeName[Mode]);
}


void measure(){

 VL53L0X_RangingMeasurementData_t measure;
  
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); 

  if (measure.RangeStatus != 4) {  
    Serial.print("Distance (mm): "); Serial.println((measure.RangeMilliMeter));
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);

}




void wealding(){

  for (int i=0; i<=24; i++) // мигание лазерным светодиодом
   {
      digitalWrite(LASER, HIGH);
      delay(50);
      digitalWrite(LASER, LOW);
      delay(50);
   }
}


void drill(){

   digitalWrite(LA, HIGH);
  delay(1000);
  digitalWrite(LA, LOW);
  delay(1000);

    digitalWrite(LB, HIGH);
  delay(1000);
  digitalWrite(LB, LOW);
  delay(1000);

}

void seek() {
  if (direct) {             // движемся в прямом направлении
    if (angle < MAX_ANGLE)
      turn_to(MAX_ANGLE);   // плавный поворот
    else {
      direct = false;       // смена направления
      delay(50);            // задержка в крайнем положении
    }
  }
  else {                    // движемся в обратном направлении
    if (angle > MIN_ANGLE)
      turn_to(MIN_ANGLE);   // плавный поворот
    else {
      direct = true;        // смена направления
      delay(50);            // задержка в крайнем положении
    }
  }
  search();                 // ищем цель дальномером
}

void hold() {
  if (!hold_flag)           // движение к середине цели
    turn_to(catch_pos);     // крутим серво
  else {                    // расчётная точка достигнута

    if (sonarTimer.isReady()) {                   // отдельный таймер сонара
      byte pos = (angle - MIN_ANGLE) / 2;         // перевод градусов в элемент Массива
      int curr_dist = sonar.ping_cm();            // получаем сигнал с датчика


      if (curr_dist == 0) curr_dist = DIST_MAX;   // 0 это не только 0, но и максимум
      int diff = distance[pos] - curr_dist;       // ищем разницу

      Serial.println("Distant = ");
Serial.print(curr_dist);
Serial.print("| Diff = ");
Serial.print(diff);
Serial.print("| Angle = ");
Serial.print(catch_pos);



      if ((diff < DEADZONE) || (curr_dist > 1 && curr_dist < 10)) {   // если вышли из зоны ИЛИ поднесли руку на 1-10 см
        if (timeoutTimer.isReady())               // отработка таймаута
          mode = true;                            // если цель потеряна и вышло время - переходим на поиск
        hold_flag = false;
      } else {                                    // если цель всё ещё в зоне видимости
        timeoutTimer.reset();                     // сбросить таймер
      }
    }
  }
}

void search() {
  if (angle % 2 == 0 && next) {                 // каждые 2 градуса
    next = false;
    byte pos = (angle - MIN_ANGLE) / 2;         // перевод градусов в элемент массива
    int curr_dist = sonar.ping_cm();
    if (curr_dist == 0) curr_dist = DIST_MAX;
    int diff = distance[pos] - curr_dist;
    if (diff > DEADZONE) {                      // разность показаний больше мертвой зоны
      if (!catch_flag) {
        catch_flag = true;        // флаг что поймали какое то значение
        catch_pos = angle;        // запомнили угол начала предполагаемой цели
      }
      catch_num++;                // увеличили счётчик значений
      if (catch_num > MIN_CATCH)  // если поймали подряд много значений
        catched_flag = true;      // считаем, что это цель
    } else {                                    // если "пусто"
      if (catch_flag) {               // если ловим цель
        if (mistakes > MISTAKES) {    // если число ошибок превысило допустимое
          // сбросить всё
          catch_flag = false;
          catched_flag = false;
          catch_num = 0;
          mistakes = 0;
        } else {
          mistakes++;                 // увеличить число ошибок
        }
      }

      if (catched_flag) {             // поймали цель! 
        mode = false;                 // переход в режим удержание
        // catch_pos - середина цели. Считаем по разному, если двигались вперёд или назад
        if (direct) catch_pos += catch_num;
        else catch_pos -= catch_num;
        
        // сбросить всё
        hold_flag = false;
        catch_flag = false;
        catched_flag = false;
        catch_num = 0;
        mistakes = 0;        
      }
    }
  }
}

void calibration() {
  // пробегаемся по рабочему диапазону
  for (angle = MIN_ANGLE; angle <= MAX_ANGLE; angle++) {
    sdrive[6].write(angle);
      if (angle % 2 == 0) {   // каждый второй градус
      byte pos = (angle - MIN_ANGLE) / 2;
      int curr_dist = sonar.ping_cm();
      if (curr_dist == 0) distance[pos] = DIST_MAX;
      else distance[pos] = sonar.ping_cm();
    }
    delay(STEP_DELAY * 1.5);
  }
}

void parce()
{
    if (_AvlDFU0)  //данные из сом порта
    {
        _AvlDFU0=0;
    }
     else
    {
        if (Serial.available()) 
        {
            _AvlDFU0=1;
            _readByteFromUART((Serial.read()),0);
        }
    }
    //Плата:1
    if (_gtv5)
    {
        if(!_RVFU2Reset)
        {
            _RVFU2Data = String("");
            _RVFU2Reset =1;
        }
    }
     else 
    {
        _RVFU2Reset =0;
    }
    if(_AvlDFU0) 
    {
        _tim2O = 1;
        _tim2I = 1;
    }
     else 
    {
         if(_tim2I) 
        {
            _tim2I = 0;
            _tim2P = millis();
        }
         else 
        {
             if (_tim2O) 
            {
                if (_isTimer(_tim2P, 20)) _tim2O = 0;
            }
        }
    }
    FTrig_2_Out = 0;
    if ((!(_tim2O))&&(FTrig_2_OldStat))
    {
        FTrig_2_Out = 1;
    }
    FTrig_2_OldStat = _tim2O;
    _gtv5 = FTrig_2_Out;
    _gtv4 = _RVFU2Data;
    //Плата:2
    _FSFS2IO = ((_gtv4).indexOf(String("X")));
    _FSFS2CO = _FSFS2IO >-1 ;
    _GSFS2 = (_gtv4).substring(((_FSFS2IO)+(1)));
    _tempVariable_String = _GSFS2;
    _convertStringToNamberOutput_7 = strtol(_tempVariable_String.c_str(),NULL,10);
    if (_FSFS2CO) 
    {
        _gtv6 = _convertStringToNamberOutput_7;
    }
    _FSFS3IO = ((_gtv4).indexOf(String("Y")));
    _FSFS3CO = _FSFS3IO >-1 ;
    _GSFS3 = (_gtv4).substring(((_FSFS3IO)+(1)));
    _tempVariable_String = _GSFS3;
    _convertStringToNamberOutput_5 = strtol(_tempVariable_String.c_str(),NULL,10);
    if (_FSFS3CO) 
    {
        _gtv7 = _convertStringToNamberOutput_5;
    }
    _FSFS1IO = ((_gtv4).indexOf(String("Z")));
    _FSFS1CO = _FSFS1IO >-1 ;
    _GSFS1 = (_gtv4).substring(((_FSFS1IO)+(1)));
    _tempVariable_String = _GSFS1;
    _convertStringToNamberOutput_1 = strtol(_tempVariable_String.c_str(),NULL,10);
    if (_FSFS1CO) 
    {
        _gtv8 = _convertStringToNamberOutput_1;
    }
    _FSFS4IO = ((_gtv4).indexOf(String("Y")));
    _FSFS4CO = _FSFS4IO >-1 ;
    _GSFS4 = (_gtv4).substring(((_FSFS4IO)+(1)));
    _tempVariable_String = _GSFS4;
    _convertStringToNamberOutput_2 = strtol(_tempVariable_String.c_str(),NULL,10);
    if (_FSFS4CO) 
    {
        _gtv9 = _convertStringToNamberOutput_2;
    }
    //Плата:3
    if (_gtv5 == 1) 
    {
        Xpos = _gtv6;
        Ypos = _gtv7;
        Zpos = _gtv8;
        Qpos = _gtv9;
        int Xp = Xpos;
        int Yp = Ypos;
        int Zp = Zpos - 50; // скорректированная координата Z  mm
        int Qp = Qpos;
        a0 = atan2(Yp, Xp)*57.3; // Угол поворота 0-й оси относительно выбранного вами 0 ° начала координат
        int Xt = sqrt(pow(Xp, 2) + pow(Yp, 2)); // Координата X целевой  точки в основной плоскости движения звеньев 60, 90 и 10
        int dX = 10*sin(Qp/57.3); // Корректировка X на ориентацию 3-го звена (10)
        int dY = 10*cos(Qp/57.3); // Корректировка Y на ориентацию 3-го звена (10)
        int Xn = Xt - dX; // Позиция X в основной плоскости, куда должен попасть шарнир 3-го звена
        int Yn = Zp + dY; // Позиция Y в основной плоскости, куда должен попасть шарнир 3-го звена
        int d = sqrt(pow(Xn, 2) + pow(Yn, 2)); // Длина отрезка между 1-м и 3-м  шарнирами
        int q1 = atan2(Yn, Xn)*57.3; // Вспомогательный угол 1
        float sq2 = (pow(d, 2) + pow(60, 2) - pow(90, 2)) / (2 * 60 * d);
        int q2 = acos(sq2)*57.3;
        float sq3 = (pow(60, 2) + pow(90, 2) - pow(d, 2)) / (2 * 60 * 90);
        int q3 = acos(sq3)*57.3;
        a1 = q1 + q2; // Угол поворота сервомотора 1-й оси относительно горизонтали
        a2 = 180 - q3; // Угол поворота сервомотора 2-й оси относительно 1-й
        a3 = 180 - q2 - q3 - q1 + Qp; // Угол поворота 3-й оси относительно перпендикуляра к 2-й оси 
        a4 = 0; // Угол поворота захвата 
        _gtv11 = a4;
        _gtv10 = a3;
        _gtv3 = a2;
        _gtv2 = a1;
        _gtv1 = a0;
    }
    //Плата:4
    if (_changeNumber3_Out) 
    {
        _changeNumber3_Out = 0;
    }
     else 
    {
        _tempVariable_int = _gtv1;
        if (_tempVariable_int != _changeNumber3_OLV) 
        {
            _changeNumber3_OLV = _tempVariable_int;
            _changeNumber3_Out = 1;
        }
    }
    if(_changeNumber3_Out) 
    {
        _tim4O = 1;
        _tim4I = 1;
    }
     else 
    {
         if(_tim4I) 
        {
            _tim4I = 0;
            _tim4P = millis();
        }
         else 
        {
             if (_tim4O) 
            {
                if (_isTimer(_tim4P, 1000)) _tim4O = 0;
            }
        }
    }
    En_57922082_2 = _tim4O;
    NewAng_57922082_2 = _gtv1;
    ServoState_57922082_2 = servo_57922082_2.tick(); // здесь происходит движение серво по встроенному таймеру!
    if(En_57922082_2)
    {
        if (millis() - servoTimer_57922082_2 >= 20) 
        {
            servoTimer_57922082_2 = millis();
            servo_57922082_2.start();
            servo_57922082_2.setTargetDeg(NewAng_57922082_2);
        }
    }
    else
    {
        servo_57922082_2.stop();
    }
    if (_changeNumber1_Out) 
    {
        _changeNumber1_Out = 0;
    }
     else 
    {
        _tempVariable_int = _gtv2;
        if (_tempVariable_int != _changeNumber1_OLV) 
        {
            _changeNumber1_OLV = _tempVariable_int;
            _changeNumber1_Out = 1;
        }
    }
    if(_changeNumber1_Out) 
    {
        _tim3O = 1;
        _tim3I = 1;
    }
     else 
    {
         if(_tim3I) 
        {
            _tim3I = 0;
            _tim3P = millis();
        }
         else 
        {
             if (_tim3O) 
            {
                if (_isTimer(_tim3P, 1000)) _tim3O = 0;
            }
        }
    }
    En_57922082_3 = _tim3O;
    NewAng_57922082_3 = _gtv2;
    ServoState_57922082_3 = servo_57922082_3.tick(); // здесь происходит движение серво по встроенному таймеру!
    if(En_57922082_3)
    {
        if (millis() - servoTimer_57922082_3 >= 20) 
        {
            servoTimer_57922082_3 = millis();
            servo_57922082_3.start();
            servo_57922082_3.setTargetDeg(NewAng_57922082_3);
        }
    }
    else
    {
        servo_57922082_3.stop();
    }
    if (_changeNumber2_Out) 
    {
        _changeNumber2_Out = 0;
    }
     else 
    {
        _tempVariable_int = _gtv3;
        if (_tempVariable_int != _changeNumber2_OLV) 
        {
            _changeNumber2_OLV = _tempVariable_int;
            _changeNumber2_Out = 1;
        }
    }
    if(_changeNumber2_Out) 
    {
        _tim1O = 1;
        _tim1I = 1;
    }
     else 
    {
         if(_tim1I) 
        {
            _tim1I = 0;
            _tim1P = millis();
        }
         else 
        {
             if (_tim1O) 
            {
                if (_isTimer(_tim1P, 1000)) _tim1O = 0;
            }
        }
    }
    En_57922082_1 = _tim1O;
    NewAng_57922082_1 = _gtv3;
    ServoState_57922082_1 = servo_57922082_1.tick(); // здесь происходит движение серво по встроенному таймеру!
    if(En_57922082_1)
    {
        if (millis() - servoTimer_57922082_1 >= 20) 
        {
            servoTimer_57922082_1 = millis();
            servo_57922082_1.start();
            servo_57922082_1.setTargetDeg(NewAng_57922082_1);
        }
    }
    else
    {
        servo_57922082_1.stop();
    }
    //Плата:5
    if (_gtv5)
    {
        if (!_stou1)
        {
            Serial.println(((String("a0: ")) + ((String(_gtv1, DEC))) + (String(" a1: ")) + ((String(_gtv2, DEC))) + (String(" a2: ")) + ((String(_gtv3, DEC))) + (String(" a3: ")) + ((String(_gtv10, DEC)))));
            _stou1=1;
        }
    }
     else 
    {
        _stou1=0;
    }
}
bool _isTimer(unsigned long startTime, unsigned long period)
{
    unsigned long currentTime;
    currentTime = millis();
    if (currentTime>= startTime) 
    {
        return (currentTime>=(startTime + period));
    }
     else 
    {
        return (currentTime >=(4294967295-startTime+period));
    }
}
void _readByteFromUART(byte data,int port)
{
    if (port==0)
    {
        _RVFU2Data += char(data);
    }
}

//  плавный поворота серво на указанный угол
void turn_to(byte to_angle) {
  if (stepTimer.isReady()) {
    if (angle < to_angle) angle++;
    else if (angle > to_angle) angle--;
    else hold_flag = true;
     sdrive[6].write(angle);
    next = true;
  }
}





























