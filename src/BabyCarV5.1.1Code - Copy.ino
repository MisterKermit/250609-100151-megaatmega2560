/*Error Docs:

avrdude: stk500v2_getsync(): timeout communicating with programmer
-select through the tools->ports-> mega adk

*/
#include <Arduino.h>
//Joystick Setup
#define VRXPIN A0
#define VRYPIN A1

#define LinacA 48
#define LinacB 46
#define LinacPosPin A3
#define ENA 12


//back and front proximity sensors
#define UltraFrontTrig 49
#define UltraFrontEcho 51

#define UltraBackTrig 45
#define UltraBackEcho 47

//Head array sensors
#define ultra_left_trig 22
#define ultra_left_echo 23

#define ultra_right_trig 24
#define ultra_right_echo 25

#define ultra_back_trig 32
#define ultra_back_echo 33
//control panel
#define UltrasonicButton 28  //ultra toggle
#define HeadArrayButton 30
#define Potentiometer A14     //motor speed
//kill switch
#define StopButton 29
#define StopLED 2

#define ForwardButton 35
#define RightButton 37
#define LeftButton 39
#define BackButton 41

#define motor1a 9
#define motor1b 8

#define LinacUpperLimit 380
#define LinacLowerLimit 220

int RealLinacPos;

//Ultrasonic Setup

int ProxFront;
long duration1;
int distance1;

int ProxBack;
long duration2;
int distance2;

int Forward;
int Right;
int Left;
int Back;

int ButtonMotorSpeed = 10;

enum head_array_state {
  LEFT,
  RIGHT,
  FRONT,
  NONE
} ;

//Killswitch Setup
int PreviousStopButtonState;
int CurrentStopButtonState;
int StopStatus;
// int StopLED = 37;

int xRestMin;
int xRestMax;
int LinacRestMax;
int LinacRestMin;
int LinacExtendMax;
int LinacExtendMin;

int yRestMin;

int yRestMax;

unsigned long t0;
unsigned long elapsed;


class Ultrasonic {
private:
  byte echo_pin;
  byte trigger_pin;
  float duration;
  int distance;
  head_array_state state;
public:
  Ultrasonic(byte e_pin, byte t_pin, head_array_state s_state) {
    echo_pin = e_pin;
    trigger_pin = t_pin;
    state = s_state;
    init();
  }

  void init() {
    pinMode(echo_pin, INPUT);
    pinMode(trigger_pin, OUTPUT);
    update();
  }

  int update() {
    
    digitalWrite(trigger_pin, LOW);
    // digitalWrite(echo_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds

    digitalWrite(trigger_pin, HIGH);
    // digitalWrite(echo_pin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigger_pin, LOW);
    // digitalWrite(echo_pin, LOW);
    delayMicroseconds(10);


    duration = pulseIn(echo_pin, HIGH, 10000);
    //duration2 = pulseIn(UltraBackEcho, HIGH);
    // Reads the echoPin, returns the sound wave travel time in microseconds

    // Calculating the distance
    distance = duration * 0.034 / 2;

    if (distance == 0) {
      distance = 1000;
    }

    return distance;
    delay(100);
  }

  int get_dist() {
    return distance;
  }

  head_array_state get_state() {
    return state;
  }

  
  bool check_prox() {
    if (digitalRead(UltrasonicButton) == HIGH) {
        return update() > 30;
    } else {
      return true;
    }
  }

  bool check_head_array() {
    if (digitalRead(HeadArrayButton) == HIGH) {
      return update() > 15;
    } else {
      return true;
    }
  }
};

void ramp_speed(int targetSpeed) {
  if (ButtonMotorSpeed < targetSpeed) ButtonMotorSpeed += 5;
}

Ultrasonic ultra_front(UltraFrontEcho, UltraFrontTrig, NONE);
Ultrasonic ultra_back(UltraBackEcho, UltraBackTrig, NONE);
Ultrasonic ultra_left(ultra_left_echo, ultra_left_trig, LEFT);
Ultrasonic ultra_right(ultra_right_echo, ultra_right_trig, RIGHT);
Ultrasonic head_ultra_back(ultra_back_echo, ultra_back_trig, FRONT);


Ultrasonic head_array[] = {ultra_left, ultra_right, head_ultra_back};


head_array_state check_head_array() {
  head_array[0].update();
  head_array[1].update();
  head_array[2].update();

  int min_value = min(min(head_array[0].get_dist(), head_array[1].get_dist()), head_array[2].get_dist());

  if (min_value >= 30 || min_value == 0) {
    return NONE;
  } else {
    for (int i = 0; i <= 2; i++) {
      if (min_value == head_array[i].get_dist()) {
        return head_array[i].get_state();
      }
    }
    return NONE;
  }
}

void setup() {
  //Motors
  pinMode(motor1a, OUTPUT);
  pinMode(motor1b, OUTPUT);

  //Linear Actuator
  pinMode(LinacA, OUTPUT);
  pinMode(LinacB, OUTPUT);
  pinMode(LinacPosPin, INPUT);

  //Ultrasonic Sensors
  pinMode(UltraFrontTrig, OUTPUT);
  pinMode(UltraFrontEcho, INPUT);

  pinMode(UltraBackTrig, OUTPUT);
  pinMode(UltraBackEcho, INPUT);

  pinMode(ultra_left_echo, INPUT);
  pinMode(ultra_left_trig, OUTPUT);

  pinMode(ultra_right_echo, INPUT);
  pinMode(ultra_right_trig, OUTPUT);

  pinMode(ultra_back_echo, INPUT);
  pinMode(ultra_back_trig, OUTPUT);

  pinMode(UltrasonicButton, INPUT);
  pinMode(HeadArrayButton, INPUT);


  //Misc
  pinMode(Potentiometer, INPUT);

  //Killswitch
  pinMode(StopButton, INPUT);
  pinMode(StopLED, OUTPUT);
  StopStatus = 0;

  //Buttons
  pinMode(ForwardButton, INPUT);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(BackButton, INPUT_PULLUP);

  Serial.begin(9600);
  Serial1.begin(9600);
}


void loop() {
  t0 = micros();
// loop logic
  elapsed = micros() - t0;
  // Serial.println(elapsed);

  //Changing Values;

  int MaxMotorSpeed = map(analogRead(Potentiometer), 0, 1023, 25, 300);  //last number value is max motor speed


  RealLinacPos = analogRead(LinacPosPin);
  // Serial.print("Linear Atuator Position:");

  //Serial.println(NegativeyValue);
  
 


  Forward = digitalRead(ForwardButton);
  Back = digitalRead(BackButton);
  Left = digitalRead(LeftButton);
  Right = digitalRead(RightButton);

  

  //Estop
  PreviousStopButtonState = CurrentStopButtonState;
  CurrentStopButtonState = (digitalRead(StopButton));

  head_array_state currentState = check_head_array();

  if ((CurrentStopButtonState == 1) && (PreviousStopButtonState == 0)) {
    if (StopStatus == 0) {
      StopStatus = 1;
      digitalWrite(StopLED, HIGH);

    } else if (StopStatus == 1) {
      StopStatus = 0;
      digitalWrite(StopLED, LOW);
    }
  }

  //Drive Controlss
  const char* haStatus = "OFF";
  const char* proxStatus = "OFF";

  bool frontProx = ultra_front.check_prox();
  bool backProx = ultra_back.check_prox();

  if (StopStatus == 0) {

    if (digitalRead(UltrasonicButton)) {
      if (!backProx) {
          proxStatus = "BACK";
      } else if (!frontProx) {
          proxStatus = "FRONT";
      } else if (!backProx && !frontProx){
          proxStatus = "BOTH";
      } else {
          proxStatus = "NONE";
      }
    } else {
        proxStatus = "OFF";
    }

    if (Forward == HIGH && frontProx) {
        Serial.println("Forward");
        ramp_speed(MaxMotorSpeed);
        analogWrite(motor1a, ButtonMotorSpeed);
        digitalWrite(motor1b, LOW);
    } else if (Back == HIGH && backProx) {
        Serial.println("Back");
        ramp_speed(MaxMotorSpeed);
        digitalWrite(motor1a, LOW);
        analogWrite(motor1b, ButtonMotorSpeed);
    } else {
      digitalWrite(motor1a, LOW);
      digitalWrite(motor1b, LOW);
      ButtonMotorSpeed = 25;
    }
    
    if (Right == HIGH && RealLinacPos < LinacUpperLimit) {
      Serial.println("Right");
      digitalWrite(LinacA, HIGH);
      digitalWrite(LinacB, LOW);
    } else if (Left == HIGH && RealLinacPos > LinacLowerLimit) {
      Serial.println("Left");
      digitalWrite(LinacB, HIGH);
      digitalWrite(LinacA, LOW);
    } else {
      digitalWrite(LinacB, LOW);
      digitalWrite(LinacA, LOW);
    }
    
    if (digitalRead(HeadArrayButton) == HIGH) {
      switch(currentState) {
        case LEFT:
          Serial.println("L");
          if (RealLinacPos > LinacLowerLimit) {
            digitalWrite(LinacB, HIGH);
            digitalWrite(LinacA, LOW);
          } else {
            digitalWrite(LinacB, LOW);
            digitalWrite(LinacA, LOW);
          }
          haStatus = "LEFT";
          break;
        case RIGHT:
          Serial.println("R");
          if (RealLinacPos <= LinacUpperLimit) {
            digitalWrite(LinacA, HIGH);
            digitalWrite(LinacB, LOW);
          } else {
            digitalWrite(LinacB, LOW);
            digitalWrite(LinacA, LOW);
          }
          haStatus = "RIGHT";
          break;
        case FRONT:
          Serial.println("F");
          if (frontProx) {
            ramp_speed(MaxMotorSpeed);
            analogWrite(motor1a, ButtonMotorSpeed);
            digitalWrite(motor1b, LOW);
            haStatus = "FRONT";
          } 
          break;
        case NONE:
          haStatus = "NONE";
          break;
      }
    } else {
      digitalWrite(motor1a, LOW);
      digitalWrite(motor1b, LOW);
      digitalWrite(LinacA, LOW);
      digitalWrite(LinacB, LOW); 
      haStatus = "OFF";
      ButtonMotorSpeed = 25;
    }
  } else {
    digitalWrite(motor1a, LOW);
    digitalWrite(motor1b, LOW);
    digitalWrite(LinacA, LOW);
    digitalWrite(LinacB, LOW); 
    haStatus = "OFF";
    proxStatus = "OFF";
    ButtonMotorSpeed = 25;
  }
 
  char sendData[200];


  
  snprintf(sendData, sizeof(sendData), "<MS: ,%i, HA: ,%s,PA: ,%s>", MaxMotorSpeed, haStatus, proxStatus);
  // Serial1.write(sendData); 
  Serial1.println(sendData);
  // snprintf(sendData, sizeof(sendData), "<%s,%i,%s,%s,%s,%s>", 1, "skib", "idi");
  // Serial1.println(sendData);

  // Send MaxMotor  Speed, Head Array Status, Prox,  
  delay(100);
}