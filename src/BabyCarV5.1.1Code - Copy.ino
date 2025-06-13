/*Error Docs:

avrdude: stk500v2_getsync(): timeout communicating with programmer
-select through the tools->ports-> mega adk

*/
#include <Arduino.h>
#include <ArrayList.h>
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
#define ultra_left_trig 13
#define ultra_left_echo 12


//control panel
#define UltrasonicButton 28  //ultra toggle
#define Potentiometer A3     //motor speed
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

// #define ultra_left_trig
// #define ultra_left_echo 

// #define ultra_right_trig 
// #define ultra_right_echo 

// #define ultra_back_trig 
// #define ultra_back_echo 


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
  BACK,
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
    Serial.begin(9600);
    update();
  }

  int update() {
    pinMode(echo_pin, INPUT);
    pinMode(trigger_pin, OUTPUT);
    digitalWrite(trigger_pin, LOW);
    // digitalWrite(echo_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds

    digitalWrite(trigger_pin, HIGH);
    // digitalWrite(echo_pin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigger_pin, LOW);
    // digitalWrite(echo_pin, LOW);

    duration = pulseIn(echo_pin, HIGH);
    //duration2 = pulseIn(UltraBackEcho, HIGH);
    // Reads the echoPin, returns the sound wave travel time in microseconds

    // Calculating the distance
    distance = duration * 0.034 / 2;
    return distance;
    delay(100);
  }

  int get_dist() {
    return distance;
  }

  
  bool check_prox() {
    if (digitalRead(UltrasonicButton) == HIGH) {
        return update() > 30;
    } else {
      return true;
    }
  }
};

void ramp_speed(int maxSpeed) {
  if (ButtonMotorSpeed < maxSpeed) {
    ButtonMotorSpeed++;
  }
}

Ultrasonic ultra_front(UltraFrontEcho, UltraFrontTrig, NONE);
Ultrasonic ultra_back(UltraBackEcho, UltraBackTrig, NONE);
// Ultrasonic ultra_left(ultra_left_echo, ultra_left_trig, LEFT);
// Ultrasonic ultra_right(ultra_right_echo, ultra_right_trig, RIGHT);
// Ultrasonic ultra_back(ultra_back_echo, ultra_back_trig, BACK);


Ultrasonic head_array[] ={};

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
  // pinMode(ultra_left_echo, INPUT);
  // pinMode(ultra_left_trig, OUTPUT);
  pinMode(UltrasonicButton, INPUT);
  ProxFront = 0;
  ProxBack = 0;


  //Misc
  pinMode(Potentiometer, INPUT);

  //Killswitch
  pinMode(StopButton, INPUT);
  pinMode(StopButton, INPUT);
  pinMode(StopLED, OUTPUT);
  StopStatus = 0;

  //Buttons
  pinMode(ForwardButton, INPUT);
  pinMode(RightButton, INPUT);
  pinMode(LeftButton, INPUT);
  pinMode(BackButton, INPUT);

  xRestMin = 375;
  xRestMax = 455;
  LinacRestMin = 50;
  LinacRestMax = 392;
  LinacExtendMax = 392;
  LinacExtendMin = 50;



  yRestMin = 512;
  yRestMax = 650;


Serial.begin(9600);

}




//ULtrasonic sensor detection


void loop() {

  //Changing Values;

  int MaxMotorSpeed = map(analogRead(Potentiometer), 0, 1023, 25, 255);  //last number value is max motor speed


  RealLinacPos = analogRead(LinacPosPin);
  // Serial.print("Linear Atuator Position:");
  Serial.println(RealLinacPos);

  //Serial.println(NegativeyValue);
  
 


  Forward = digitalRead(ForwardButton);
  Back = digitalRead(BackButton);
  Left = digitalRead(LeftButton);
  Right = digitalRead(RightButton);







  // Serial.println(inputs[0].value);
  // ProxBack = check_prox(ultra_back);
  // ProxFront = check_prox(ultra_front);


  // distance_values[0] = my_Ultrasonics[0].check_prox();
  // distance_values[1] = my_Ultrasonics[1].check_prox();
  // distance_values[2] =


  // float distance_chosen = min(min(distance_values[0], distance_values[1]), distance_values[2]);
  //int distance_chosen = distance_values[0];
  
  // Serial.println(distance_chosen);
  // put threshold
  /*
  if (distance_chosen < 10 && digitalRead(UltrasonicButton) == 1) {
    if (distance_chosen == distance_values[0] && ProxFront == 0) {
      head_array_state = LEFT;
      Serial.println("left");
    if (RealLinacPos > LinacExtendMin) {
      digitalWrite(LinacA, HIGH);
      digitalWrite(LinacB, LOW);
    }
    } else if (distance_chosen == distance_values[1]) {
      head_array_state = BACK;
      Serial.println("front");
    } else if (distance_chosen == distance_values[2]) {
      head_array_state = RIGHT;
      Serial.println("right");
    } else {
      head_array_state = NONE;
      Serial.println("none");
    }
  } else {
    // Serial.println("join the glorius ovulation");
  }
  // Serial.println(ProxFront);

*/

  // //End of Linear Actuator No Input Commands
  // else if ((xValue > RealLinacPos) && (xValue > xRestMax) && (StopStatus == 0)) {
  //   digitalWrite(LinacA, HIGH);
  //   digitalWrite(LinacB, LOW);
  // }
  // else if ((xValue < RealLinacPos) && (xValue < xRestMin) && (StopStatus == 0)) {
  //   digitalWrite(LinacA, LOW);
  //   digitalWrite(LinacB, HIGH);
  // }
  // else {
  //   digitalWrite(LinacA, LOW);
  //   digitalWrite(LinacB, LOW);
  // }

  //Estop
  PreviousStopButtonState = CurrentStopButtonState;
  CurrentStopButtonState = (digitalRead(StopButton));

  if ((CurrentStopButtonState == 1) && (PreviousStopButtonState == 0)) {
    if (StopStatus == 0) {
      StopStatus = 1;
    } else if (StopStatus == 1) {
      StopStatus = 0;
    }
  }
  if (StopStatus == 1) {
    digitalWrite(StopLED, HIGH);
  } else if (StopStatus == 0) {
    digitalWrite(StopLED, LOW);
  }

  //Drive Controls

  if (StopStatus == 0) {
    if (Forward == HIGH && ultra_front.check_prox()) {
        Serial.println("Forward");
        ramp_speed(MaxMotorSpeed);
        analogWrite(motor1a, ButtonMotorSpeed);
        digitalWrite(motor1b, LOW);  //WIP

      } else if (Right == HIGH && RealLinacPos < LinacUpperLimit) {
        Serial.println("Right");
        digitalWrite(LinacA, HIGH);
        digitalWrite(LinacB, LOW);
          //  Serial.println("WORk");
      } else if (Left == HIGH && RealLinacPos > LinacLowerLimit) {
        Serial.println("Left");
        digitalWrite(LinacB, HIGH);
        digitalWrite(LinacA, LOW);
      } else if (Back == HIGH && (ultra_back.check_prox())) {
        Serial.println("Back");
        ramp_speed(MaxMotorSpeed);
        digitalWrite(motor1a, LOW);
        analogWrite(motor1b, ButtonMotorSpeed);
      } else {
      
        digitalWrite(motor1a, LOW);
        digitalWrite(motor1b, LOW);
        digitalWrite(LinacA, LOW);
        digitalWrite(LinacB, LOW);
      
        // analogWrite(i, 1);
        ButtonMotorSpeed = 25;
      }
  }



 

  
  delay(100);
}
