/********** Included Libraries **********/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <LiquidCrystal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

/********** Macros **********/
// Bit Manipulations
#define SET_MASK(reg, mask)         (reg) |= (mask)
#define CLEAR_MASK(reg, mask)       (reg) &= ~(mask)
#define SET_BIT(reg, pin)			(reg) |= (1 << (pin))
#define TOGGLE_BIT(reg, pin)		(reg) ^= (1 << (pin))
#define CLEAR_BIT(reg, pin)			(reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)	(reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)			(((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)		(BIT_VALUE((reg),(pin))==1)

// Serial Definitions
#define BAUD                        (9600)
#define MYUBRR                      (F_CPU/16/BAUD-1)
#define TX_BUFFER_SIZE              (64)
#define RX_BUFFER_SIZE              (64)

// Timer Definitions
#define PRESCALE                    (8.0)
#define DC_SN                       (((39/2) + 25) * 0.01)

// LCD Pins
#define B1 9
#define B0 8
#define D4 4
#define D5 5
#define D6 6
#define D7 7

/********** Initialisation of Variables **********/
// Constant Duty Cycle (Dependant on Student Number)
uint8_t DC = 256*DC_SN;

// Initialize the LCD
uint8_t rs = B1, en = B0;
LiquidCrystal lcd(rs, en, D4, D5, D6, D7);

// Bitmap for Degrees in Celsius
uint8_t bmp1[8] = { 0b00000000, 
                    0b00010111,
                    0b00000100,
                    0b00000100,
                    0b00000111,
                    0b00000000,
                    0b00000000,
                    0b00000000};

// Timer Definitions
volatile int overflow_counter = 0;

/********** Functions Declaration **********/
uint16_t adc( uint8_t channel );
void process(void);
void setup(void);
void setup_adc( uint8_t division_factor );
void setup_lcd(void);
void setup_timers(void);
void setup_uart(unsigned int ubrr);
void uart_putchar(uint8_t c);
void uart_putstring(char* s);

/********** Functions **********/
int main(void) {
    setup();
    while(1) {
        process();
        _delay_ms(500);
    }
}

// Obtains Analogue Values Using ADC
uint16_t adc ( uint8_t channel ) {
    CLEAR_MASK(ADMUX, 0b1111);
    SET_MASK(ADMUX, channel);
    // Start Single Conversion, Set ADSC
    SET_BIT(ADCSRA, ADSC);
    // Wait for Conversion to Complete
    while (ADCSRA & (1 << ADSC));
    return (ADC);
}

// Main Process
void process(void) {
  	// Assist in Debouncing of Button Press
  	uint8_t pb_now = 0, pb_prev = 0, pb_pressed;
  	pb_now = ~PINB;
	pb_pressed = pb_now & (pb_now ^ pb_prev);
	pb_prev = pb_now;
    // Char Data to be Sent to Serial/LCD
    char temperature[TX_BUFFER_SIZE];
    char lcd_time[6]={'\0'};
    // Obtain Real Time 
    double time = (overflow_counter * 65536.0 + TCNT1 ) * PRESCALE  / F_CPU;
    // Obtain Raw Temperature
    uint16_t temp = adc(0);
    // Convert Raw Analog Signal to Temperature
    temp = (temp * 165/338) - (8410/169);
    // Check Extreme Conditions (0 < temp < 50 )
    if (temp >= 50) {
        // Turn On Alarm LED
        SET_BIT(PORTB, PINB5);
        // Set PWM Duty Cycle to 44.5%
        OCR2B = DC;
        // Check Putton Press for Blue LED
        if ( BIT_IS_SET(PINB, PINB3) & pb_pressed ) {
            // Turn On Blue LED
            TOGGLE_BIT(PORTB, PINB4);
        }
    } else {
        // LEDs Turn Off
        CLEAR_BIT(PORTB, PINB5);
        CLEAR_BIT(PORTB, PINB4); 
        // PWM to 0% Output
        OCR2B = 0;
    }
    // Convert Temperature from uint16_t to String
    snprintf(temperature, sizeof(temperature), "%d", temp);
    // Convert Time from Double to String
    dtostrf(time, 5, 2, lcd_time);
    // Display Temp & Time on LCD
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.write(1); 
    lcd.setCursor(0,1);
    lcd.print("Timer:");
    lcd.print(lcd_time);
}

// Initialise ADC, LCD, Timers and UART
void setup(void) {
    // Set Input B3 (Push Button)
    CLEAR_BIT(DDRB, PINB4);
    // Set Output B5 & B4 (Red & Blue LED)
    SET_BIT(DDRB, PINB4);
    SET_BIT(DDRB, PINB5);
    // Output is set for Peizo Buzzer
    SET_BIT(DDRD, PIND3);
    // Enable ADC with Pre-Scale Division Factor 128. 
    setup_adc(128);
    // Setup LCD
    setup_lcd();
    // Setup Timers
    setup_timers();
    //init uart
    setup_uart(MYUBRR);
}

// Setup ADC
void setup_adc( uint8_t division_factor ) {
    uint8_t prescalar = 7;
    WRITE_BIT(ADMUX, REFS1, 0);
    WRITE_BIT(ADMUX, REFS0, 1);
    WRITE_BIT(ADCSRA, ADEN, 1);
    while(!BIT_IS_SET(division_factor, prescalar) && prescalar > 1) {
        prescalar--;
    }
    CLEAR_MASK(ADCSRA, 0b111 << ADPS0);
    SET_MASK(ADCSRA, prescalar << ADPS0);
}

// Setup LCD
void setup_lcd(void) {
    // Creating Celsius Symbol from Bitmap 1
    lcd.createChar(1, bmp1);  
    // Setup LCD in 4-Pin Mode
    lcd.begin(16,2);
    // Print Initial Message
    lcd.print("Temperature");
    lcd.setCursor(0,1);
    lcd.print("Sensor");
    _delay_ms(1000);
}

// Setup Timers
void setup_timers(void) {
    // Timer 1 in normal mode, with pre-scaler 8 ==> ~0.9 Hz overflow.
    // and overflow interrupt
  	SET_BIT(TCCR1B, CS11);
    SET_BIT(TIMSK1, TOIE1);
    // Timer 2 Compare Register Initially 0
    OCR2B = 0;
    // Set Non-Inverting Mode
    SET_BIT(TCCR2A, COM2B1);
    // Set Prescaler to 8, Starts PWM
    SET_BIT(TCCR2B, CS21);
    // Set Fast PWM Mode
    SET_BIT(TCCR2A, WGM20);
    SET_BIT(TCCR2A, WGM21);
    // Enable Timer Overflow, Turn On Interrupts.
	sei();
}

// Timer 1 Overflow Interrupt 
ISR(TIMER1_OVF_vect) {
	overflow_counter++;
}


// Initialize the UART
void setup_uart(unsigned int ubrr) {
  	cli();  
  
  	UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)(ubrr);
    SET_BIT(UCSR0B, RXEN0);
    SET_BIT(UCSR0B, TXEN0);
  	SET_BIT(UCSR0B, RXCIE0);
    SET_BIT(UCSR0C, UCSZ00);
  	SET_BIT(UCSR0C, UCSZ01);

	sei();
}

// Transmits a Byte
void uart_putchar(unsigned char data) {
    // Wait for Transmit Buffer
    while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/
    // Data's Put into Buffer, Data's Sent
    UDR0 = data;
}

// Transmits a String
void uart_putstring(char* s) {
    // Transmit Until NULL is Reached
    while(*s > 0) uart_putchar(*s++);
}

// Receive Interrupt
ISR(USART_RX_vect) {
  	char temperature[RX_BUFFER_SIZE];
  	//UDR0 = UDR0;
    // Obtain Raw Temperature
    uint16_t temp = adc(0);
    // Convert Raw Analog Signal to Temperature
    temp = (temp * 165/338) - (8410/169);
  	// Convert Temperature from uint16_t to String
    snprintf(temperature, sizeof(temperature), "%d", temp);
  	uart_putstring((char *) temperature);
 	uart_putstring((char *) " Degrees");
    uart_putstring((char *) "\r\n");
  	UDR0;
}
