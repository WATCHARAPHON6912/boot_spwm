
#include <avr/io.h>
#include <avr/interrupt.h>

#define en_boot PD4
#define pwm_boot 5

#define en_spwm PD2
#define _2H PB2  //spwm
#define _2L PB1
#define _1H PB3
#define _1L PD3

#define status 12
#define ifb A0
#define boot_fb A1


// int data=65*2;
int data = 65;

#define f_xtel 16000000
// #define f_xtel 32000000
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

  Serial.begin(115200);

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

  TCCR0A = 1 << COM0B1 | 1 << WGM00;
  TCCR0B = 1 << CS00;
  HDR = 1 << HDR0;
  DDRD = 1 << DDD5;
  OCR0B = 0;


  sei(); /* Enable global interrupts.*/
  interrupts();
  DDRB = 0b00011110; /* Pin 9, 10, 11, 12 as outputs.*/
  PORTB = 0;
  Amplitude = 0.01;

  DDRD |= (1 << en_boot);
  DDRD |= (1 << en_spwm);

  DDRB |= (1 << _2H);
  DDRB |= (1 << _2L);
  DDRB |= (1 << _1H);
  DDRD |= (1 << _1L);

  PORTD &= ~(1 << en_spwm);
  PORTD &= ~(1 << en_boot);
}



int start = 0;
int d = 0;
int li = 2;
// unsigned long period = 25;         //ระยะเวลาที่ต้องการรอ
unsigned long period = 400;   //ระยะเวลาที่ต้องการรอ
unsigned long last_time = 0;  //ประกาศตัวแปรเป็น global เพื่อเก็บค่าไว้ไม่ให้ reset จากการวนloop
// unsigned long period_boot = 8000;  //ระยะเวลาที่ต้องการรอ
unsigned long period_boot = 800;  //ระยะเวลาที่ต้องการรอ
unsigned long last_time_boot = 0;
double add_sub = 0.002;
double min = 0.10;
double max = 0.80;
int duty_boot = 0;
// int boot_li = 80;
int boot_li = map(50, 0, 100, 0, 255);
void loop() {

  check();
  if (start <= 0) {
    delay(15000);
    start++;
    PORTD |= (1 << en_spwm);
    PORTD |= (1 << en_boot);


    int volt_boot = analogRead(boot_fb);
    int boot_set = 5;
    for (int dd = 0; dd <= 1000; dd++) {
      if (millis() - last_time_boot > period_boot) {
        last_time_boot = millis();  //เซฟเวลาปัจจุบันไว้เพื่อรอจนกว่า millis() จะมากกว่าตัวมันเท่า period
        if (volt_boot < boot_set - 3) duty_boot += 1;
        if (volt_boot > boot_set + 3) duty_boot -= 1;

        if (duty_boot >= boot_li) duty_boot = boot_li;
        if (duty_boot <= 0) duty_boot = 0;
        OCR0B = duty_boot;
      }
    }
  }

  //########################################################################################################################
  int volt_boot = analogRead(boot_fb);
  // int boot_set = 200;
  int boot_set = 10;

  Serial.print(pid_control(boot_set, volt_boot));
  Serial.print("\t");

  if (volt_boot < boot_set) duty_boot += 1;
  // if (volt_boot > boot_set) duty_boot -= 1;
  if (volt_boot > boot_set) duty_boot += pid_control(boot_set, volt_boot);

  if (duty_boot >= boot_li) duty_boot = boot_li;
  if (duty_boot <= 0) duty_boot = 0;
  // analogWrite(pwm_boot, map(duty_boot, 0, 100, 0, data));
  OCR0B = duty_boot;

  Serial.print(volt_boot);
  Serial.print("\t");
  Serial.println(duty_boot);
  //########################################################################################################################

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
  int dt = 30;
  static int dtA = dt;
  static int dtB = dt;
  if (num == point / 2 && ph != 1) {
    Vo = analogRead(0);
  }
  if (num >= point - 1) {   // <------------------ 50 Hz !!!
    if (ph == 0) {          // OC1A as SPWM out
      TCCR1A = 0b10110000;  // clear OC1A, set OC1B on compare match
      dtA = 0;              // no dead time
      dtB = dt;             // adding dead time to OC1B
    } else {
      TCCR1A = 0b11100000;  // OC1B as SPWM out
      dtA = dt;
      dtB = 0;
    }
    ph ^= 1;
  }
  OCR1A = int(set_sin[num] * Amplitude) + dtA;  // SPWM width update
  OCR1B = int(set_sin[num] * Amplitude) + dtB;  // note: 0.7 used to reduce inveter output voltage
  // OCR1A=clk/2+dtA;
  // OCR1B=clk/2+dtB;
  num++;
  if (num >= point) {
    if (f_spwm <= 10000) {
      delayMicroseconds(60);
    }
    if (ph == 1) {
      PORTD &= ~(1 << _1L);
      // delayMicroseconds(50);
      PORTB |= (1 << _1H);
      phs = 1;
    } else {
      PORTB &= ~(1 << _1H);
      // delayMicroseconds(50);
      PORTD |= (1 << _1L);
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
  PORTD &= ~(1 << en_spwm);
  PORTD &= ~(1 << en_boot);

  while (1) {
    int lop = 20000;
    digitalWrite(status, 1);
    delay(lop);
    digitalWrite(status, 0);
    delay(lop);
  }
}


int pid_control(int setpoint, int measured_value) {
  // # กำหนดค่า PID gains
  float Kp = 1.0;
  float Ki = 0.1;
  float Kd = 0.0;
  // # กำหนดค่าสำหรับ PID controller
  float error_sum = 0.0;
  float last_error = 0.0;

  float error = setpoint - measured_value;
  float P_term = Kp * error;
  error_sum += error;
  float I_term = Ki * error_sum;
  float D_term = Kd * (error - last_error);

  last_error = error;
  int control_signal = P_term + I_term + D_term;

  return control_signal;
}

// setpoint = 50
// measured_value = 50
// control_signal = pid_control(setpoint, measured_value)
// print("Control signal:", control_signal)