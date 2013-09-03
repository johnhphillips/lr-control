// define input pins that monitor control box, three binary digits
#define CONTROL_A0 4
#define CONTROL_A1 5
#define CONTROL_A2 6

// define output pins that drive motor controllers
#define WINCH_AHI 7
#define WINCH_BHI 8
#define WINCH_PWM 9

#define TROLLEY_AHI 12
#define TROLLEY_BHI 13
#define TROLLEY_FORE_PWM 10
#define TROLLEY_AFT_PWM 11

#define WINCH_CURRENT A0

// define PWM values
#define zero_pwr 0
#define half_pwr 25
#define full_pwr 225

volatile int control_input = 0;
volatile int prev_control_input = 0;

volatile int winch_current = 0;
volatile int prev_winch_current = 0;

volatile int trolley_fore_current = 0;
volatile int prev_trolley_fore_current = 0;
volatile int trolley_aft_current = 0;
volatile int prev_trolley_aft_current = 0;


void setup()  { 
  // Setup serial output to port (debugging)
  Serial.begin(9600);
  // Setup control box input, active low
  pinMode(CONTROL_A0, INPUT);
  pinMode(CONTROL_A1, INPUT);
  pinMode(CONTROL_A2, INPUT);
  
  // Turn on pullup resistors for control box input
  digitalWrite(CONTROL_A0, HIGH);
  digitalWrite(CONTROL_A1, HIGH);
  digitalWrite(CONTROL_A2, HIGH);
 
  pinMode(WINCH_CURRENT, INPUT);
  
  // Setup outputs to winch motor control
  pinMode(WINCH_AHI, OUTPUT);
  pinMode(WINCH_BHI, OUTPUT);  
  pinMode(WINCH_PWM, OUTPUT);
  // Setup outputs to trolley motor controls
  pinMode(TROLLEY_AHI, OUTPUT);
  pinMode(TROLLEY_BHI, OUTPUT);
  pinMode(TROLLEY_FORE_PWM, OUTPUT);
  pinMode(TROLLEY_AFT_PWM, OUTPUT);
} 

void loop()  { 
  // read control input 
  control_input = PIND & B01110000;
  
  if( control_input != prev_control_input) {
    Serial.print(control_input, BIN);
    Serial.print("\n");
    switch ( control_input){
      case B01100000:    // Winch up at full speed
        analogWrite(WINCH_PWM, full_pwr);
        digitalWrite(WINCH_AHI, HIGH);    
        Serial.print("WINCH UP\n");
        break;
       
      case B01010000:    // Winch up at half speed
        analogWrite(WINCH_PWM, half_pwr);
        digitalWrite(WINCH_AHI, HIGH);
        Serial.print("WINCH UP SLOW\n");
        break; 
      
      case B01000000:    // Winch down at full speed
        analogWrite(WINCH_PWM, full_pwr);
        digitalWrite(WINCH_BHI, HIGH);
        Serial.print("WINCH DOWN\n");
        break;
      
      case B00110000:    // Winch down at half speed
        analogWrite(WINCH_PWM, half_pwr);
        digitalWrite(WINCH_BHI, HIGH);
        Serial.print("WINCH DOWN SLOW\n");
        break;
        
      default:          // Winch disabled
        analogWrite(WINCH_PWM, zero_pwr);
        digitalWrite(WINCH_AHI, LOW);
        digitalWrite(WINCH_BHI, LOW);
        Serial.print("SYSTEM IDLE\n");
        break;
    }
    // move current to previous before next loop
    prev_control_input = control_input;
    }
    winch_current = analogRead(WINCH_CURRENT);
    if( winch_current != prev_winch_current) {
      Serial.print(winch_current);
      Serial.print("\n");
    }
    prev_winch_current = winch_current;
}

