/*Error Docs:

avrdude: stk500v2_getsync(): timeout communicating with programmer
-select through the tools->ports-> mega adk

*/
#include <Arduino.h>
#include <ArrayList.h>
//Joystick Setup
#define VRXPIN A0
#define VRYPIN A1

#define LinacA 26
#define LinacB 27
#define LinacPosPin A3
#define ENA 12


//back and front proximity sensors
#define UltraFrontTrig 52
#define UltraFrontEcho 50

#define UltraBackTrig 53
#define UltraBackEcho 51

//Head array sensors
#define ultra_left_trig 13
#define ultra_left_echo 12

// #define ultra_right_trig
// #define ultra_right_echo

// #define ultra_fender_trig
// #define ultra_right_echo

//control panel
#define UltrasonicButton 31  //ultra toggle
#define Potentiometer A3     //motor speed
//kill switch
#define StopButton 35
#define StopLED 37

#define ForwardButton 35
#define RightButton 37
#define LeftButton 39
#define BackButton 41

#define motor1a 9
#define motor1b 8

#define LinacUpperLimit 380
#define LinacLowerLimit 194


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
}  head_array_state;

//change values
// int ultra_left_trig =
// int ultra_left_echo =

// int ultra_right_trig =
// int ultra_right_echo =

// int ultra_fender_trig =
// int ultra_fender_echo =


// int UltrasonicButton = 36;

//Misc Setup
// int Potentiometer = A3;

//Killswitch Setup
// int StopButton = 35;
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
public:
  Ultrasonic(byte e_pin, byte t_pin) {
    echo_pin = e_pin;
    trigger_pin = t_pin;
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

  
  bool proximity_fb() {
    return update() < 40;
  }
};

Ultrasonic ultra_front(UltraFrontEcho, UltraFrontTrig);
Ultrasonic ultra_back(UltraBackEcho, UltraBackTrig);
// Ultrasonic ultra_left(ultra_left_echo, ultra_left_trig);

Ultrasonic my_Ultrasonics[] = { ultra_front, ultra_back};



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
  // pinMode(UltrasonicButton, INPUT);
  ProxFront = 0;
  ProxBack = 0;

  //Misc
  pinMode(Potentiometer, INPUT);

  //Killswitch
  // pinMode(StopButton, INPUT);
  // pinMode(StopLED, OUTPUT);
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

void ramp_speed(int maxSpeed) {
  if (ButtonMotorSpeed < maxSpeed) {
    ButtonMotorSpeed++;
  }
}


//ULtrasonic sensor detection
// int check_prox(Ultrasonic sensor) {
//   // float ultra_distance;
//   if (digitalRead(UltrasonicButton) == 1) {
//     if (sensor.proximity_fb()) {
//       if (i == 0) {
//         ProxFront = 1;
//         return ProxFront;
//       } else {
//         ProxBack = 1;
//         return ProxBack;
//       }
//     } else {
//       if (i == 0) {
//         ProxFront = 0;
//         return ProxFront;
//       } else {
//         ProxBack = 0;
//         return ProxBack;
//       }
//     }
//   } else {
//     return 0;
//   }
// }

void loop() {

  //Changing Values;

  int MaxMotorSpeed = map(analogRead(Potentiometer), 0, 1023, 25, 255);  //last number value is max motor speed


  // RealLinacPos = analogRead(LinacPosPin);
  //Serial.print("Linear Actuator Position:");
  // Serial.println(RealLinacPos);

  //Serial.println(NegativeyValue);
  
 


  Forward = digitalRead(ForwardButton);
  Back = digitalRead(BackButton);
  Left = digitalRead(LeftButton);
  Right = digitalRead(RightButton);




  // Serial.println(inputs[0].value);
  // ProxBack = check_prox(ultra_back);
  // ProxFront = check_prox(ultra_front);

  // for (int i = 0; i <= 3; i++) {
  //   inputs[i].value = digitalRead(inputs[i].buttonPin);
  //   check_button(inputs[i].motorPin, inputs[i].value, MaxMotorSpeed);
  // }
// int distance_values[] = { my_Ultrasonics[2].get_dist() };


  // distance_values[0] = my_Ultrasonics[0].check_prox();
  // distance_values[1] = my_Ultrasonics[1].check_prox();
  // distance_values[2] =

  // for (int j = 0; j < 2; j++) {
  //   distance_values[j] = my_Ultrasonics[j].check_prox();
  //   // Serial.println(distance_values[j]);
  // }

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

  

  if ((Forward == HIGH)) {
    Serial.println("Forward");
    ramp_speed(MaxMotorSpeed);
    analogWrite(motor1a, ButtonMotorSpeed);
    digitalWrite(motor1b, LOW);  //WIP

  } else if ((Right == HIGH)) {
    Serial.println("Right");
    if (RealLinacPos < LinacUpperLimit) {
      digitalWrite(LinacA, HIGH);
      digitalWrite(LinacB, LOW);
    }
      //  Serial.println("WORk");
  } else if ((Left == HIGH)) {
    Serial.println("Left");
    if (RealLinacPos > LinacLowerLimit) {
      digitalWrite(LinacB, HIGH);
      digitalWrite(LinacA, LOW);
    }
  } else if (Back == HIGH) {
    Serial.println("Back");
    ramp_speed(MaxMotorSpeed);
    digitalWrite(motor1a, LOW);
    analogWrite(motor1b, ButtonMotorSpeed);
  } else {
    digitalWrite(motor1a, LOW);
    digitalWrite(motor1b, LOW);
    // analogWrite(i, 1);
    ButtonMotorSpeed = 25;
  }



  //head array
  // Ultrasonic ultra_right(ultra_right_echo, ultra_right_trig);
  // Ultrasonic ultra_fender(ultra_fender_echo, ultra_fender_trig);


  //change pins

  // Serial.println(ProxFront);
  // Serial.println(ProxBack);
  // Serial.println(digitalRead(UltrasonicButton));

  



  //Killswitch
  // PreviousStopButtonState = CurrentStopButtonState;
  // CurrentStopButtonState = (digitalRead(StopButton));

  // if ((CurrentStopButtonState == 1) && (PreviousStopButtonState == 0)) {
  //   if (StopStatus == 0) {
  //     StopStatus = 1;
  //   } else if (StopStatus == 1) {
  //     StopStatus = 0;
  //   }
  // }
  // if (StopStatus == 1) {
  //   digitalWrite(StopLED, HIGH);
  // } else if (StopStatus == 0) {
  //   digitalWrite(StopLED, LOW);
  // }

  
  delay(100);
}
