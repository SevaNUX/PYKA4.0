float Xp, Yp, Zp;// Переменные для хранения введённых значений

void setup() {
  Serial.begin(9600); // Настройка скорости передачи данных через последовательный порт
  while(!Serial);     // Ждём подключение монитора серийного порта
  Serial.println("Введите значения X и Y, Z");
}

void loop() {
  String inputStrX, inputStrY, inputStrZ; // Строки для временного хранения ввода
  float tempValue;            

  // Ожидание ввода значения X
  while(Serial.available() <= 0); 
  inputStrX = Serial.readStringUntil('\n'); // Читаем строку до нажатия Enter
  tempValue = inputStrX.toFloat();          // Преобразуем строку во float
  Xp = tempValue;                            // Сохраняем значение в переменную x
  Serial.print("Значение X установлено: "); Serial.println(Xp);

  // Ожидание ввода значения Y
  while(Serial.available() <= 0); // Ждем новые данные
  inputStrY = Serial.readStringUntil('\n');
  tempValue = inputStrY.toFloat();
  Yp = tempValue;
  Serial.print("Значение Y установлено: "); Serial.println(Yp);

    while(Serial.available() <= 0); 
  inputStrZ = Serial.readStringUntil('\n'); 
  tempValue = inputStrX.toFloat();          
  Zp = tempValue;                            
  Serial.print("Значение Z установлено: "); Serial.println(Zp);

  
  performCalculations();   
}

void performCalculations() {
  float result = Xp * Yp* Zp; 
 Serial.print("Результат умножения: "); Serial.println(result);
}