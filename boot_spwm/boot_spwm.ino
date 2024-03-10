
#include <avr/io.h>
#include <avr/interrupt.h>

#define en_boot 4
#define pwm_boot 5

#define en_spwm 2
#define _2H 10  //spwm
#define _2L 9
#define _1H 11
#define _1L 3

#define status 12
#define ifb A0
#define boot_fb A1




#define f_xtel 16000000
#define f_spwm 23000
#define f_sin 50

#define Cycle (f_spwm / f_sin)
#define point1 Cycle / 2
int point = point1;
int clk = (f_xtel / f_spwm) / 2;
double Amplitude;
int phs;
int Vo;
int set_sin[point1 + 1] = {};

void setup() {
  pinMode(ifb, INPUT);
  pinMode(boot_fb, INPUT);

  for (int i = 2; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }
  digitalWrite(en_spwm, LOW);
  digitalWrite(en_boot, LOW);
  Serial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  double angle;

  for (int i = 0; i <= point; i++) {
    angle = i * M_PI / point;
    set_sin[i] = int(sin(angle) * clk);
    Serial.print(i);
    Serial.print(" ");
    Serial.println(set_sin[i]);
  }
  Serial.println(sizeof(set_sin));


  // Register initilisation, see datasheet for more detail.
  TCCR1A = 0b10110000;
  TCCR1B = 0b00010001;
  TIMSK1 = 0b00000001;
  ICR1 = clk; /* Counter TOP value (at 16MHz XTAL, SPWM carrier freq. 10kHz, 200 samples/cycle).*/

  // TCCR0B = 0;
  // TCCR0B |= (1 << CS00);
  TCCR0A = _BV(COM0A0) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
  TCCR0B = _BV(WGM02) | _BV(CS01);
  OCR0A = 65;  //30khz


  sei();             /* Enable global interrupts.*/
  DDRB = 0b00011110; /* Pin 9, 10, 11, 12 as outputs.*/
  PORTB = 0;
  Amplitude = 0.01;
}



int start = 0;
int d = 0;
int li = 2;
unsigned long period = 25;         //ระยะเวลาที่ต้องการรอ
unsigned long last_time = 0;       //ประกาศตัวแปรเป็น global เพื่อเก็บค่าไว้ไม่ให้ reset จากการวนloop
unsigned long period_boot = 5000;  //ระยะเวลาที่ต้องการรอ
unsigned long last_time_boot = 0;
double add_sub = 0.002;
double min = 0.10;
double max = 0.80;
int duty_boot = 0;
int boot_li = 50;
void loop() {
  check();
  if (start <= 0) {
    delay(15000);
    start++;
    digitalWrite(en_spwm, HIGH);
    digitalWrite(en_boot, HIGH);
  }
  if (millis() - last_time_boot > period_boot) {
    last_time_boot = millis();  //เซฟเวลาปัจจุบันไว้เพื่อรอจนกว่า millis() จะมากกว่าตัวมันเท่า period
    //########################################################################################################################
    int volt_boot = analogRead(boot_fb);
    int boot_set = 500;
    if (volt_boot + boot_li <= boot_set) duty_boot += 1;
    if (volt_boot - boot_li >= boot_set) duty_boot -= 1;

    if (duty_boot >= 50) duty_boot = 50;
    // if (duty_boot >= 80) duty_boot = 80;
    if (duty_boot <= 1) duty_boot = 1;

    Serial.print(volt_boot);
    Serial.print("\t");
    Serial.println(duty_boot);
    analogWrite(pwm_boot, map(duty_boot, 0, 100, 0, 65));

    //########################################################################################################################
  }
  if (millis() - last_time > period) {
    last_time = millis();  //เซฟเวลาปัจจุบันไว้เพื่อรอจนกว่า millis() จะมากกว่าตัวมันเท่า period

    Vo = 10;
    int volt_set = 20;

    // Serial.print(volt_set);
    // Serial.print(" ");
    // Serial.print(Vo);
    // Serial.print(" ");
    // Serial.print(Amplitude);
    // Serial.println(" ");

    if ((Vo + li) < volt_set) {
      Amplitude = Amplitude + add_sub;
    } else if ((Vo - li) > volt_set) {
      Amplitude = Amplitude - add_sub;
    }
    if (Amplitude <= min) Amplitude = min;
    if (Amplitude >= max) Amplitude = max;
    //########################################################################################################################
  }
}

ISR(TIMER1_OVF_vect) {

  static int num;
  static int ph;
  static int dtA = 5;
  static int dtB = 5;
  if (num == point / 2 && ph != 1) {
    Vo = analogRead(0);
  }
  if (num >= point - 1) {   // <------------------ 50 Hz !!!
    if (ph == 0) {          // OC1A as SPWM out
      TCCR1A = 0b10110000;  // clear OC1A, set OC1B on compare match
      dtA = 0;              // no dead time
      dtB = 5;              // adding dead time to OC1B
    } else {
      TCCR1A = 0b11100000;  // OC1B as SPWM out
      dtA = 5;
      dtB = 0;
    }
    ph ^= 1;
  }
  OCR1A = int(set_sin[num] * Amplitude) + dtA;  // SPWM width update
  OCR1B = int(set_sin[num] * Amplitude) + dtB;  // note: 0.7 used to reduce inveter output voltage

  num++;
  if (num >= point) {
    if (f_spwm <= 10000) {
      delayMicroseconds(60);
    }
    if (ph == 1) {
      digitalWrite(_1L, LOW);
      // delayMicroseconds(100);
      digitalWrite(_1H, HIGH);
      phs = 1;
    } else {
      digitalWrite(_1H, LOW);
      // delayMicroseconds(100);
      digitalWrite(_1L, HIGH);
      phs = 0;
    }
    num = 0;
  }
}
void check() {
  if (digitalRead(ifb) == 1) {
    while (1) {
      alarmIndication(5);
    }
  }
}
unsigned long p = 1000;  //ระยะเวลาที่ต้องการรอ
unsigned long l = 0;     //ประกาศตัวแปรเป็น global เพื่อเก็บค่าไว้ไม่ให้ reset จากการวนloop
void alarmIndication(int alarm) {
  TCCR1A = 0;  // shutdown SPWM output
  TIMSK1 = 0;
  PORTB &= 0b11100001;

  digitalWrite(en_boot, LOW);
  digitalWrite(en_spwm, LOW);


  while (1) {
    int lop = 20000;
    digitalWrite(status, 1);
    delay(lop);
    digitalWrite(status, 0);
    delay(lop);
  }
}