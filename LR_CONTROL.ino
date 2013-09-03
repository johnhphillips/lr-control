// Define digital input pins that monitor control box
#define WINCH_UP        2
#define WINCH_UP_SLOW   3
#define WINCH_DOWN      4
#define WINCH_DOWN_SLOW 5

// Define output pins that drive motor controllers
#define WINCH_AHI     6
#define WINCH_BHI     7
#define WINCH_PWM     9

#define MC_DISABLE    A1

// Define static PWM values
#define zero_pwr 0
#define half_pwr 127
#define full_pwr 225

// Define other variables
int control_input = 0;
int prev_control_input = 0;

// Initial microcontroller setup
void setup() {
  Serial.begin(9600);
  pinMode(WINCH_UP, INPUT);
  pinMode(WINCH_UP_SLOW, INPUT);
  pinMode(WINCH_DOWN, INPUT);
  pinMode(WINCH_DOWN_SLOW, INPUT);
  
  pinMode(WINCH_AHI, OUTPUT);
  pinMode(WINCH_BHI, OUTPUT);
  pinMode(WINCH_PWM, OUTPUT);
  pinMode(MC_DISABLE, OUTPUT);
}
void loop()  { 
  // check control box switches binary to decimal
  // 0001 = WINCH_UP = 1
  // 0010 = WINCH_UP_SLOW = 2
  // 0100 = WINCH_DOWN = 4
  // 1000 = WINCH_DOWN_SLOW = 8
  control_input = digitalRead(WINCH_UP) + (digitalRead(WINCH_UP_SLOW) << 1) + 
  (digitalRead(WINCH_DOWN) << 2) + (digitalRead(WINCH_DOWN_SLOW) << 3);
  // check current control input against previous control input
  if( control_input != prev_control_input) {
    switch( control_input) {
     case 1:                          // WINCH_UP
      digitalWrite(WINCH_AHI, HIGH);
      analogWrite(WINCH_PWM, full_pwr);
      Serial.print("UP!\n");
      break;
    
     case 2:                          // WINCH_UP_SLOW
      digitalWrite(WINCH_AHI, HIGH);
      analogWrite(WINCH_PWM, half_pwr);
      Serial.print("UP SLOW!\n");
      break;
     
     case 4:                          // WINCH_DOWN
      digitalWrite(WINCH_BHI, HIGH);
      analogWrite(WINCH_PWM, full_pwr);
      Serial.print("DOWN!\n");
      break;
      
     case 8:                          // WINCH_DOWN_SLOW
      digitalWrite(WINCH_BHI, HIGH);
      analogWrite(WINCH_PWM, half_pwr);
      Serial.print("DOWN SLOW!\n");
      break;
     
     default:
      digitalWrite(WINCH_AHI, LOW);
      digitalWrite(WINCH_BHI, LOW);
      analogWrite(WINCH_PWM, 0);
      Serial.print("IDLE\n");
    }
  }
  // change previous input then loop again
  prev_control_input = control_input;
}

void read_input() {
  
}

