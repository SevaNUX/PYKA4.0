#define IR_1    0xA2
#define IR_2    0x62
#define IR_3    0xE2
#define IR_4    0x22
#define IR_5    0x2
#define IR_6    0xC2
#define IR_7    0xE0
#define IR_8    0xA8
#define IR_9    0x90
#define IR_STAR 0x68
#define IR_0    0x98
#define IR_HASH 0xB0
#define IR_UP   0x18
#define IR_LEFT 0x10
#define IR_OK   0x38
#define IR_RIGHT 0x5A
#define IR_DOWN 0x4A

#include <NecDecoder.h>
NecDecoder ir;

void setup() {
  Serial.begin(9600);
  attachInterrupt(0, irIsr, FALLING);
}

void irIsr() {
  ir.tick();
}

void loop() {
  if (ir.available()) {
    switch (ir.readCommand()) {
      case IR_1: Serial.println("Pressed 1"); break;
      case IR_2: Serial.println("Pressed 2"); break;
      case IR_3: Serial.println("Pressed 3"); break;
      case IR_4: Serial.println("Pressed 4"); break;
      case IR_5: Serial.println("Pressed 5"); break;
      case IR_6: Serial.println("Pressed 6"); break;
      case IR_7: Serial.println("Pressed 7"); break;
      case IR_8: Serial.println("Pressed 8"); break;
      case IR_9: Serial.println("Pressed 9"); break;
      case IR_STAR: Serial.println("Pressed *"); break;
      case IR_0: Serial.println("Pressed 0"); break;
      case IR_HASH: Serial.println("Pressed #"); break;
      case IR_UP: Serial.println("Pressed up"); break;
      case IR_LEFT: Serial.println("Pressed left"); break;
      case IR_OK: Serial.println("Pressed ok"); break;
      case IR_RIGHT: Serial.println("Pressed right"); break;
      case IR_DOWN: Serial.println("Pressed down"); break;
    }
  }
}