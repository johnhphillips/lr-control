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
volatile boolean fault = LOW;
volatile boolean winch_active = LOW;

volatile int current_input = 0;
volatile int prev_input = 0;
volatile long last_time = 0;

volatile int winch_current = 0;

void timer1_init()
{
  cli();                   // global interrupt disable

  // setup the 16-bit timer 1 
  // TCCR1A - Timer1 / Counter Control Register A
  TCCR1A = 0;              // Set register to 0
  // TCCR1B - Timer1 / Counter Control Register B
  TCCR1B = 0;              // Set register to 0
  TCNT1 = 0;               // Initialize counter value to 0
  // set compare match register for 10000Hz increments (0.1 msec)
  OCR1A = 800 - 1;           
  TCCR1B |= (1 << WGM12);  // Turn on CTC mode
  // Set CS10 bit for no prescaler
  TCCR1B |= (1 << CS10);
  // enable timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);          

  sei();                     // global interrupt enable
  return; 
}
void adc_init()
{
  ADCSRA = 0;
  ADCSRA |= ((1 << ADPS2)|(1 << ADPS0));//|(1 << ADPS0));                   // 8MHZ/2 = 4MHz the ADC reference clk
  ADMUX |= (1 << REFS0);                    // Voltage reference from Avcc 3.3V
  ADCSRA |= (1 << ADEN);                    // Turn on ADC
  ADCSRA |= (1 << ADSC);                    // Perform initial converesion
}

void setup()  
{ 
  // Setup serial output for debugging
  Serial.begin(9600);
  // Setup ADC for current sensor sampling
  adc_init();
  // Setup Timer1 for 0.1ms sample rate
  timer1_init();
  
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

void read_input() 
{
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
  return;
}

void process_input()
{
  // check that not faulted, then process input 
 if( !fault) {
   switch ( current_input) {
     case B01100000:    // Winch up at full speed
       analogWrite(WINCH_PWM, FULL_PWR);
       digitalWrite(WINCH_AHI, HIGH);    
//       Serial.println("WINCH UP");
       break;
         
     case B01010000:    // Winch up at half speed
       analogWrite(WINCH_PWM, HALF_PWR);
       digitalWrite(WINCH_AHI, HIGH);
//       Serial.println("WINCH UP SLOW");
       break; 
        
     case B01000000:    // Winch down at full speed
       analogWrite(WINCH_PWM, FULL_PWR);
       digitalWrite(WINCH_BHI, HIGH);
//       Serial.println("WINCH DOWN");
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
//       Serial.println("SYSTEM IDLE");
       break;
    }
  }
  return;
}

void loop() {
  // read control input 
  read_input(); 
  // process control input
  process_input();
 
  Serial.println(winch_current);
  if( winch_current > OVERCURRENT && !fault) {
    // diable timer1 compare interrupt
    TIMSK1 &= ~(1 << OCIE1A);
    digitalWrite(FAULT_A0, HIGH);
    fault = HIGH;
  } 
}

// Timer1 interrupt service routine
ISR(TIMER1_COMPA_vect) 
{
  winch_current = analogRead(WINCH_CURRENT);            // read state of winch current
  digitalWrite( FAULT_A0, !digitalRead(FAULT_A0));    // used for debugging
}

