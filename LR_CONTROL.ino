// Input pins that monitor control box, three binary digits
#define CONTROL_A0 4
#define CONTROL_A1 5
#define CONTROL_A2 6

// Output pins that drive winch motor controller
#define WINCH_AHI 7
#define WINCH_BHI 8
#define WINCH_PWM 9
// Output pins that drive fore and aft trolley motor controllers
#define TROLLEY_AHI 12
#define TROLLEY_BHI 13
#define TROLLEY_FORE_PWM 10
#define TROLLEY_AFT_PWM 11

#define MC_DISABLE A3

// ADC input pins that monitor motor current / enclosure temperature
#define WINCH_CURRENT A0
#define TROLLEY_FORE_CURRENT A1
#define TROLLEY_AFT_CURRENT A2
#define OVERTEMP A4

/* Output pins that send fault code to interface 
   000 = NO FAULT
   001 = WINCH OVERCURRENT
   010 = FORE TROLLEY OVERCURRENT
   011 = AFT TROLLEY OVERCURRENT
   100 = ENCLOSURE OVERTEMPERATURE
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
volatile int fault = 0;
volatile int winch_active = 0;

// Temporary input variables to micro
volatile int control_input = 0;
volatile int prev_control_input = 0;

volatile int winch_current = 0;
volatile int prev_winch_current = 0;

volatile int fault_code = B000;
volatile int prev_fault_code = B000;

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



void loop()  { 
  // read control input, pins 4, 5, 6
  control_input = PIND & B01110000;
  // check that input has changed, ADD DEBOUNCE, and that not faulted
  if( control_input != prev_control_input && fault != 1) {
    Serial.print(control_input, BIN);
    Serial.print("\n");
    switch ( control_input){
      case B01100000:    // Winch up at full speed
        winch_active = 1;
        analogWrite(WINCH_PWM, FULL_PWR);
        digitalWrite(WINCH_AHI, HIGH);    
        Serial.println("WINCH UP");
        break;
       
      case B01010000:    // Winch up at half speed
        winch_active = 1;
        analogWrite(WINCH_PWM, HALF_PWR);
        digitalWrite(WINCH_AHI, HIGH);
        Serial.println("WINCH UP SLOW");
        break; 
      
      case B01000000:    // Winch down at full speed
        winch_active = 1;
        analogWrite(WINCH_PWM, FULL_PWR);
        digitalWrite(WINCH_BHI, HIGH);
        Serial.println("WINCH DOWN");
        break;
      
      case B00110000:    // Winch down at half speed
        winch_active = 1;
        analogWrite(WINCH_PWM, HALF_PWR);
        digitalWrite(WINCH_BHI, HIGH);
        Serial.println("WINCH DOWN SLOW");
        break;
        
      default:          // Winch disabled
        winch_active = 0;
        analogWrite(WINCH_PWM, ZERO_PWR);
        digitalWrite(WINCH_AHI, LOW);
        digitalWrite(WINCH_BHI, LOW);
        Serial.println("SYSTEM IDLE");
        break;
    }
    // move current to previous before next loop
    prev_control_input = control_input;
    }
        // in fault state
    else if( fault == 1 && fault_code != prev_fault_code) {
      // disable all motor controller outputs
      winch_active = 0;
      analogWrite(WINCH_PWM, ZERO_PWR);
      digitalWrite(WINCH_AHI, LOW);
      digitalWrite(WINCH_BHI, LOW);
      Serial.println("SYSTEM FAULT");
      // send fault code to output
      digitalWrite(FAULT_A0, fault_code);
      Serial.print("FAULT CODE = ");
      Serial.println(fault_code, BIN);
      prev_fault_code = fault_code;
    }
    // read in analog voltage that represents winch current, 
    // only when winch is active
    if( winch_active) { 
      winch_current = analogRead(WINCH_CURRENT);
      // debugging info sent to serial output
      if( winch_current != prev_winch_current) {
//        Serial.print("ADC Value = ");
//        Serial.print(winch_current);
//        Serial.print("\n");
        Serial.print("Voltage = ");
        double voltage = (winch_current * 3.3) / 1023;
        Serial.print(voltage);
        Serial.print("\n");
      }
      // check that winch current is in an overcurrent state
      if( winch_current <= NEG_OVERCURRENT || winch_current >= POS_OVERCURRENT) {
        digitalWrite(MC_DISABLE, HIGH);
        fault = 1;
        fault_code = B001;
      }
      prev_winch_current = winch_current;
    }
    
}

