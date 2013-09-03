// define input pins that monitor control box
#define WINCH_UP 2
#define WINCH_UP_SLOW 3
#define WINCH_DOWN 4
#define WINCH_DOWN_SLOW 5

// define output pins that drive motor controllers
#define WINCH_PWM 9 

// define PWM values
#define zero_pwr 0
#define half_pwr 127
#define full_pwr 225

volatile int control_input = 0;
volatile int prev_control_input = 0;

void setup()  { 
  Serial.begin(9600);
  // Setup control box input, active low
  pinMode(WINCH_UP, INPUT);
  pinMode(WINCH_UP_SLOW, INPUT);
  pinMode(WINCH_DOWN, INPUT);
  pinMode(WINCH_DOWN_SLOW, INPUT);
  // Turn on pullup resistors for control box input
  digitalWrite(WINCH_UP, HIGH);
  digitalWrite(WINCH_UP_SLOW, HIGH);
  digitalWrite(WINCH_DOWN, HIGH);
  digitalWrite(WINCH_DOWN_SLOW, HIGH);
  
  pinMode(WINCH_PWM, OUTPUT);
  
} 

void loop()  { 
  // read control input (pin 2)
  control_input = PIND & B00111100;
  
  if( control_input != prev_control_input) {
//    Serial.print(control_input, BIN);
//    Serial.print("\n");
    switch ( control_input){
      case B00111000:    // Winch up at full speed
        analogWrite(WINCH_PWM, full_pwr);    
        Serial.print("WINCH UP\n");
        break;
       
      case B00110100:    // Winch up at half speed
        analogWrite(WINCH_PWM, half_pwr);
        Serial.print("WINCH UP SLOW\n");
        break; 
      
      case B00101100:    // Winch down at full speed
        analogWrite(WINCH_PWM, full_pwr);
        Serial.print("WINCH DOWN\n");
        break;
      
      case B00011100:    // Winch down at half speed
        analogWrite(WINCH_PWM, half_pwr);
        Serial.print("WINCH DOWN SLOW\n");
        break;
        
      default:          // Winch disabled
        analogWrite(WINCH_PWM, zero_pwr);
        Serial.print("SYSTEM IDLE\n");
        break;
    }
    // move current to previous before next loop
    prev_control_input = control_input;
    }
}

