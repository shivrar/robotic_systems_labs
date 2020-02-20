// These #defines act like a find-and-replace
// in your code, and make your code more readable.
// Note that there is no #define for E0_B:.
// it's a non-standard pin, check out setupEncoder0().
#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26
#define MIN_RES 0.0044; //Constant in radians for minimum readable distance between encoder ticks

//~~~~~~~~~~~~~~~~~Important variables to be avaiable to other functions~~~~~~~~~~~~~~~~~~~~~~~//
volatile float theta_e1;
volatile float theta_e0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//


// Volatile Global variables used by Encoder ISR.
volatile bool oldE0_A;  // used by encoder to remember prior state of A
volatile bool oldE0_B;  // used by encoder to remember prior state of B
volatile bool oldE1_A;  // used by encoder to remember prior state of A
volatile bool oldE1_B;  // used by encoder to remember prior state of B
//TODO: NEXT STEPS: Try to get the FK up and running 

// This ISR handles just Encoder 1
// ISR to read the Encoder1 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( INT6_vect ) {
  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  bool newE1_B = digitalRead( E1_B_PIN );
  bool newE1_A = digitalRead( E1_A_PIN );

  newE1_A ^= newE1_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( newE1_A  << 3 );
  state = state | ( newE1_B  << 2 );
  state = state | ( oldE1_A  << 1 );
  state = state | ( oldE1_B  << 0 );

  switch( state ) {
    case 1:  theta_e1-=MIN_RES; break;  // clockwise?
    case 2:  theta_e1+=MIN_RES; break;  // anti-clockwise?
    case 4:  theta_e1+=MIN_RES; break;  // anti-clockwise?
    case 7:  theta_e1-=MIN_RES;  break;  // clockwise?
    case 8:  theta_e1-=MIN_RES;  break;  // clockwise?
    case 11: theta_e1+=MIN_RES; break;  // anti-clockwise?
    case 13: theta_e1+=MIN_RES;  break;  // anti-clockwise?
    case 14: theta_e1-=MIN_RES;  break;  // clockwise?
    default: break; //No movement or invalid cases we do nothing
  }

  // Save current state as old state for next call.
  oldE1_A = newE1_A;
  oldE1_B = newE1_B;
}


// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( PCINT0_vect ) {

  // First, Read in the new state of the encoder pins.

  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.  
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  bool newE0_B = PINE & (1<<PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  bool newE0_A = digitalRead( E0_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the 
  // true value.
  newE0_A ^= newE0_B;


  
  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;                   
  state = state | ( newE0_A  << 3 );
  state = state | ( newE0_B  << 2 );
  state = state | ( oldE0_A  << 1 );
  state = state | ( oldE0_B  << 0 );

  switch( state ) {
    case 1:  theta_e0-=MIN_RES; break;  // clockwise?
    case 2:  theta_e0+=MIN_RES; break;  // anti-clockwise?
    case 4:  theta_e0+=MIN_RES; break;  // anti-clockwise?
    case 7:  theta_e0-=MIN_RES;  break;  // clockwise?
    case 8:  theta_e0-=MIN_RES;  break;  // clockwise?
    case 11: theta_e0+=MIN_RES; break;  // anti-clockwise?
    case 13: theta_e0+=MIN_RES;  break;  // anti-clockwise?
    case 14: theta_e0-=MIN_RES;  break;  // clockwise?
    default: break; //No movement or invalid cases we do nothing
  }
     
  // Save current state as old state for next call.
  oldE0_A = newE0_A;
  oldE0_B = newE0_B; 
}

void setupEncoder1() {

  theta_e1 = 0.0;
  oldE1_A = 0;
  oldE1_B = 0;

  pinMode( E1_A_PIN, INPUT );
  pinMode( E1_B_PIN, INPUT );

  EIMSK = EIMSK & ~(1<<INT6);
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
  EIFR |= ( 1 << INTF6 );
  EIMSK |= ( 1 << INT6 );
}

void setupEncoder0() {

    theta_e0 = 0.0;
    oldE0_A = 0;
    oldE0_B = 0;
    DDRE = DDRE & ~(1<<DDE6);
    PORTE = PORTE | (1<< PORTE2 );

    pinMode( E0_A_PIN, INPUT );
    digitalWrite( E0_A_PIN, HIGH ); // Encoder 0 xor

    PCICR = PCICR & ~( 1 << PCIE0 );
    PCMSK0 |= (1 << PCINT4);
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.
    PCICR |= (1 << PCIE0);
}
