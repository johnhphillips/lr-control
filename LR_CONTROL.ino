// Input pins that monitor control box, three binary digits
#define CONTROL_A0 4
#define CONTROL_A1 5
#define CONTROL_A2 6

// Output pins that drive winch motor controller
#define WINCH_AHI 7
#define WINCH_BHI 8
#define WINCH_PWM 9

#define MC_DISABLE A3

// ADC input pins that monitor motor current / enclosure temperature
#define WINCH_CURRENT A0

/* Output pins that send fault code to interface 
   000 = NO FAULT
   001 = WINCH OVERCURRENT
*/
#define FAULT_A0 A5
#define FAULT_A1 A6
#define FAULT_A2 A7

/* Motor current boundries 
   992 = 3.3V
   341 = 1.1V
*/
#define POS_OVERCURRENT 992
#define NEG_OVERCURRENT 341 

#define DEBOUNCE_TIME 10

// PWM values
#define ZERO_PWR 0
#define HALF_PWR 25
#define FULL_PWR 225

// State flags, all initially false
volatile byte fault = LOW;
volatile byte winch_active = LOW;

volatile int current_input;
volatile int prev_input = 0;
volatile long last_time = 0;

void setup()  { 
  // Setup serial output for debugging
  Serial.begin(9600);
  
  // Control box input, active low
  pinMode(CONTROL_A0, INPUT);
  pinMode(CONTROL_A1, INPUT);
  pinMode(CONTROL_A2, INPUT);
  
  // Turn on pullup resistors for control box input
  digitalWrite(CONTROL_A0, HIGH);
  digitalWrite(CONTROL_A1, HIGH);
  digitalWrite(CONTROL_A2, HIGH);
  
  // Winch current monitor 
  pinMode(WINCH_CURRENT, INPUT);
  
  // Outputs to winch motor control
  pinMode(WINCH_AHI, OUTPUT);
  pinMode(WINCH_BHI, OUTPUT);  
  pinMode(WINCH_PWM, OUTPUT);
  
  pinMode(MC_DISABLE, OUTPUT);
  
  // Outputs for fault information, to interface
  pinMode(FAULT_A0, OUTPUT);
  pinMode(FAULT_A1, OUTPUT);
  pinMode(FAULT_A2, OUTPUT);
} 

void read_input() {
  // Read the state of the control inputs into a local variable:
  int reading = PIND & B01110000;

  // If the input has changed, due to noise or pressing:
  if ( reading != prev_input) {
    // Reset the debouncing timer
    last_time = millis();
  } 
  
  if ( (millis() - last_time) > DEBOUNCE_TIME) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    current_input = reading;
  }  
  prev_input = reading;
}

void loop() {
 // read control input 
 read_input(); 
 // check that not faulted, then process input 
 if( !fault) {
 switch ( current_input){
      case B01100000:    // Winch up at full speed
        analogWrite(WINCH_PWM, FULL_PWR);
        digitalWrite(WINCH_AHI, HIGH);    
//        Serial.println("WINCH UP");
        break;
       
      case B01010000:    // Winch up at half speed
        analogWrite(WINCH_PWM, HALF_PWR);
        digitalWrite(WINCH_AHI, HIGH);
//        Serial.println("WINCH UP SLOW");
        break; 
      
      case B01000000:    // Winch down at full speed
        analogWrite(WINCH_PWM, FULL_PWR);
        digitalWrite(WINCH_BHI, HIGH);
//        Serial.println("WINCH DOWN");
        break;
      
      case B00110000:    // Winch down at half speed
        analogWrite(WINCH_PWM, HALF_PWR);
        digitalWrite(WINCH_BHI, HIGH);
//       Serial.println("WINCH DOWN SLOW");
        break;
        
      default:          // Winch disabled
        analogWrite(WINCH_PWM, ZERO_PWR);
        digitalWrite(WINCH_AHI, LOW);
        digitalWrite(WINCH_BHI, LOW);
//        Serial.println("SYSTEM IDLE");
        break;
    }
 }
}

