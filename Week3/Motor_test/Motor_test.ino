// Pin definitions, to make the
// code easier to read.
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

// Variables to remember our
// motor power for Left and Right.
// Byte stores 0 to 255
byte l_power;
byte r_power;

// Setup, only runs once when powered on.
void setup() {

  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, LOW);
  digitalWrite( R_DIR_PIN, LOW);
  
  // Set initial l_power and r_power values.
  l_power = 0;
  r_power = 0;

  // Start up the Serial for debugging.
  Serial.begin(9600);
  delay(1000);
  // Print reset so we can catch any reset error.
  Serial.println(" ***Reset*** ");

}


// put your main code here, to run repeatedly:
void loop() {

  // Adjust power. e.g., increment by 4 on every loop()
  l_power = l_power + 4;
  r_power = r_power + 4;

  // Send power PWM to pins, to motor drivers.
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );

  // Brief pause
  delay(250);

}
