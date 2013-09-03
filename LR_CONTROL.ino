#include <avr/io.h>
#include <avr/interrupt.h>

// define macros used to increase code readability
#define SET(x,y) (x |= (1 << y))
#define CLR(x,y) (x &= (~(1 << y)))
#define CHK(x,y) (x & (1 << y))
#define TOG(x,y) (x ^= (1 << y))

// Input pins that monitor control box, three binary digits
#define CONTROL_A0 4
#define CONTROL_A1 5
#define CONTROL_A2 6

// Output pins that drive winch motor controller
#define WINCH_AHI PORTD7            // PORTD bit 7, IDE pin 7, pin 13
#define WINCH_BHI PORTB0            // PORTB bit 0, IDE pin 8, pin 14
#define WINCH_PWM PORTD3            // PORTD bit 3, IDE pin 3, pin 5 / Timer2 - OC2B

// Output pinss that drive the trolley motor controllers
#define TROLLEY_AHI PORTB2          // PORTB bit 2, IDE pin 10, pin 16
#define TROLLEY_BHI PORTB4          // PORTB bit 4, IDE pin 12, pin 18
#define FWD_TROLLEY_PWM PORTB3      // PORTB bit 3, IDE pin 11, pin 17 / Timer2 - OC2A
#define AFT_TROLLEY_PWM PORTD3      // Same pin is used for winch and aft trolley PWM

#define MC_DISABLE A3

// ADC input pins that monitor motor current / enclosure temperature
#define WINCH_CURRENT A0
#define FWD_TROLLEY_CURRENT A1
#define AFT_TROLLEY_CURRENT A2
#define ENC_TEMP A4

/* Output pins that send fault code to interface 
   000 = NO FAULT
   001 = WINCH OVERCURRENT
   010 = FWD TROLLEY OVERCURRENT
   011 = AFT TROLLEY OVERCURRENT
   100 = ENCLOSURE OVERTEMP
   101 = CONTROLLER FAILURE
*/
#define FAULT_A0 A5
#define FAULT_A1 A6
#define FAULT_A2 A7

/* Motor current boundries 
   997 ~= 3.2V
*/
#define OVERCURRENT 997

// 10 msec debounce time
#define DEBOUNCE_TIME 10

// PWM values
#define ZERO_PWR 0
#define HALF_PWR 25
#define FULL_PWR 255

// State flags, all initially false
int fault = 0;
int winch_active = 0;
int trolley_active = 0;

// input variables used for debouncing and control box input
volatile int current_input = 0;
volatile int prev_input = 0;
volatile long last_time = 0;

volatile int winch_current = 0;
volatile int fwd_trolley_current = 0;
volatile int aft_trolley_current = 0;
volatile int enc_temp = 0;

// function used to initialize Timer0 and Timer2
static inline void timer_init()
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
  // setup the 8-bit timer 2 
  // TCCR2A - Timer1 / Counter Control Register A
  TCCR2A = 0;              // Set register to 0
  // TCCR2B - Timer1 / Counter Control Register B
  TCCR2B = 0;              // Set register to 0
  TCNT2 = 0;               // Initialize counter value to 0
  // set non-inverting mode for A and B outputs
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1);
  // set fast PWM mode
  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  // set prescaler to 8 and starts PWM (1MHz)
  TCCR2B |= (1 << CS21); 
  // set compare match registers for zero duty cycle
  OCR2A = ZERO_PWR;
  OCR2B = ZERO_PWR;
  
  sei();                     // global interrupt enable
  return; 
}
// function used to initialize ADC
static inline void adc_init()
{
  ADCSRA = 0;
  ADCSRA |= ((1 << ADPS2)|(1 << ADPS0));    // 8MHZ/2 = 4MHz the ADC reference clk
  ADMUX |= (1 << REFS0);                    // Voltage reference from Avcc 3.3V
  ADCSRA |= (1 << ADEN);                    // Turn on ADC
  ADCSRA |= (1 << ADSC);                    // Perform initial converesion
}

// function used to read ADC
uint16_t read_adc( uint8_t channel)
{
  channel &= B00000111;                     // AND with 7 to ensure input channel is valid (0 - 7)
  ADMUX &= 0x0F8;                           // Clear old channel that was read
  ADMUX |= channel;                         // Defines new channel to read 
  ADCSRA |= (1 << ADSC);                    // Start new conversion
  while( ADCSRA & (1 << ADSC));             // Wait until conversion is complete 
  return ADC;  
}

void setup()  
{ 
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
  
  // Analog input monitors
  pinMode(WINCH_CURRENT, INPUT);
  pinMode(FWD_TROLLEY_CURRENT, INPUT);
  pinMode(AFT_TROLLEY_CURRENT, INPUT);
  pinMode(ENC_TEMP, INPUT);
    
  // Outputs to winch motor control
  SET(DDRD, WINCH_AHI);
  SET(DDRB, WINCH_BHI);
  SET(DDRD, WINCH_PWM);
  
  // Outputs to trolley motor controllers
  SET(DDRB, TROLLEY_AHI);
  SET(DDRB, TROLLEY_BHI);
  SET(DDRB, FWD_TROLLEY_PWM);
  
  pinMode(MC_DISABLE, OUTPUT);
  
  // Outputs for fault information, to interface
  pinMode(FAULT_A0, OUTPUT);
  pinMode(FAULT_A1, OUTPUT);
  pinMode(FAULT_A2, OUTPUT);
  
    // Setup ADC for current sensor sampling
  adc_init();
  // Setup Timer for 0.1ms sample rate and Timer2 for PWM
  timer_init();
} 

// function used to read input from control box via priority encoder
void read_input() 
{
//  digitalWrite( 13, !digitalRead(13));              // used for debugging 
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

// function used to process input from control box
void process_input()
{
  // check that not faulted, then process input 
 if( !fault) {
   switch ( current_input) {
     case B01100000:    // Winch up at full speed
       winch_active = 1;
       SET( TCCR2A, COM2B1);
       OCR2B = FULL_PWR;
       SET( PORTD, WINCH_AHI);
       break;
         
     case B01010000:    // Winch up at half speed
       winch_active = 1;
       SET( TCCR2A, COM2B1);
       OCR2B = HALF_PWR;
       SET( PORTD, WINCH_AHI);
       break; 
        
     case B01000000:    // Winch down at full speed
       winch_active = 1;
       SET( TCCR2A, COM2B1);
       OCR2B = FULL_PWR;
       SET( PORTB, WINCH_BHI);
       break;
        
     case B00110000:    // Winch down at half speed
       winch_active = 1;
       SET( TCCR2A, COM2B1);
       OCR2B = HALF_PWR;
       SET( PORTB, WINCH_BHI);
       break;
       
     case B00100000:   // FWD & AFT trolley port
       trolley_active = 1;
       SET( TCCR2A, COM2A1);
       SET( TCCR2A, COM2B1);
       OCR2A = FULL_PWR;
       OCR2B = FULL_PWR;
       SET( PORTB, TROLLEY_AHI);
       break;
       
     case B00010000:   // FWD & AFT trolley stbd
       trolley_active = 1;
       SET( TCCR2A, COM2A1);
       SET( TCCR2A, COM2B1);
       OCR2A = FULL_PWR;
       OCR2B = FULL_PWR;
       SET( PORTB, TROLLEY_BHI);
       break;
                
     default:          // Winch & trolleys disabled
       winch_active = 0;
       trolley_active = 0;
       CLR( TCCR2A, COM2A1);
       CLR( TCCR2A, COM2B1);
       CLR( PORTD, WINCH_PWM);
       CLR( PORTD, WINCH_AHI);
       CLR( PORTB, WINCH_BHI);
       CLR( PORTB, FWD_TROLLEY_PWM);
       CLR( PORTB, TROLLEY_AHI);
       CLR( PORTB, TROLLEY_BHI);
       break;
    }
  }
  else {              // Winch & trolleys disabled
    winch_active = 0;
    trolley_active = 0;
    CLR( TCCR2A, COM2A1);
    CLR( TCCR2A, COM2B1);
    CLR( PORTD, WINCH_PWM);
    CLR( PORTD, WINCH_AHI);
    CLR( PORTB, WINCH_BHI);
    CLR( PORTB, FWD_TROLLEY_PWM);
    CLR( PORTB, TROLLEY_AHI);
    CLR( PORTB, TROLLEY_BHI);
    }
  digitalWrite( 13, !digitalRead(13));              // used for debugging
  return;
}
// function used to check status of winch / trolley motor currents 
void check_currents() {
  // check that winch is not in overcurrent condition
  if( winch_current > OVERCURRENT && !fault) {
    // diable timer1 compare interrupt
    CLR( TIMSK1, OCIE1A);
    // disable motor controllers 
    digitalWrite(MC_DISABLE, HIGH);
    fault = 1;
  } 
  // check that fwd trolley is not in overcurrent condition
  if( fwd_trolley_current > OVERCURRENT && !fault) {
    // diable timer1 compare interrupt
    CLR( TIMSK1, OCIE1A);
    // disable motor controllers 
    digitalWrite(MC_DISABLE, HIGH);
    fault = 1;
  } 
  // check that aft trolley is not in overcurrent condition
  if( aft_trolley_current > OVERCURRENT && !fault) {
    // diable timer1 compare interrupt
    CLR( TIMSK1, OCIE1A);
    // disable motor controllers 
    digitalWrite(MC_DISABLE, HIGH);
    fault = 1;
  }
  return;
}

void loop() {
  // read control input 
  read_input(); 
  // process control input
  process_input();
  // check currents of active motors
  check_currents();
  
//  digitalWrite( 13, !digitalRead(13));              // used for debugging 
}

// Timer1 interrupt service routine
ISR(TIMER1_COMPA_vect) 
{    
  if( winch_active) {   
    winch_current = read_adc(PORTC0);                 // read state of winch current
//    digitalWrite( 13, !digitalRead(13));              // used for debugging
  }
  if( trolley_active) {
    fwd_trolley_current = read_adc(1);           // read state of fwd trolley current
//    aft_trolley_current = read_adc(2);           // read state of aft trolley current
//    digitalWrite( 13, !digitalRead(13));              // used for debugging
  }
//  enc_temp = read_adc(PORTC4);                      // read state of enclosure temperature
}
