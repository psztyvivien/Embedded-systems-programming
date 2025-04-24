//libraries
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

//analog input
#define MoistureSensorPin 0  // ADC0 (PC0)

//relay: a switch operated by electromagnetic current
#define motorRelayPin 1      // Digital pin 9 (PB1)

#define bt_set  3            // A3 (PC3)
#define bt_up   4            // A4 (PC4)
#define bt_down 5            // A5 (PC5)

//LCD Pins
#define LCD_RS 2             // Digital pin 2 (PD2)
#define LCD_EN 3             // Digital pin 3 (PD3)
#define LCD_D4 4             // Digital pin 4 (PD4)
#define LCD_D5 5             // Digital pin 5 (PD5)
#define LCD_D6 6             // Digital pin 6 (PD6)
#define LCD_D7 7             // Digital pin 7 (PD7)

//TIMER2 !!!! 
uint8_t counter = 0;
uint16_t szamlalo=0;

//USART
#define FOSC 16000000 //órajel sebesség
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1
unsigned char be[20] ={};
unsigned char nedvesseg[4]={};


//moisture threshold defaults (adjustable)
float setL_moisture = 30.0; // min
float setH_moisture = 70.0; // max

int Set = 0, flag = 0, flash = 0;

//Timer variables
unsigned long lastLcdUpdate = 0; // Last time LCD was updated
const unsigned long lcdUpdateInterval = 1000; // Update LCD every 1000ms

//functions
void timer_init();
void led_init();
void lcd_init();
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_print(const char* str);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_clear();
uint16_t adc_read(uint8_t channel);
void eeprom_write(uint8_t address, uint8_t value);
uint8_t eeprom_read(uint8_t address);
void delay_ms(uint16_t ms);
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);
void USART_string(unsigned char *s);

void setup_registers() {
  
  DDRD |= (1 << LCD_RS) | (1 << LCD_EN) | (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7); // LCD pins as output
  DDRB |= (1 << motorRelayPin);  // Motor relay pin as output
  DDRC &= ~((1 << bt_set) | (1 << bt_up) | (1 << bt_down)); // Buttons as input
  PORTC |= (1 << bt_set) | (1 << bt_up) | (1 << bt_down);   // Enable pull-ups on buttons

  //ADC
  ADMUX = (1 << REFS0);             
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

  //EEPROM inic.
  if (eeprom_read(0) == 0xFF) {
    eeprom_write(10, (uint8_t)setL_moisture);
    eeprom_write(15, (uint8_t)setH_moisture);
    eeprom_write(0, 0x00);
  }
  setL_moisture = eeprom_read(10);
  setH_moisture = eeprom_read(15);

  lcd_init();  // Init. LCD
}

//interruprt service routine
ISR(TIMER2_OVF_vect)  //triggers every ~16.384ms
{
    if(szamlalo==30)
    {
      szamlalo=0;
      //LED blinking
      PORTB ^= (1<<PB5); 
    }
    else counter++;
}


void setup() {
  USART_Init(MYUBRR);
  USART_string((unsigned char *)"Intelligens\n\rOntozorendszer\n\r");
  
	led_init();
  setup_registers();
  timer_init();
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Intelligens");
  lcd_set_cursor(0, 1);
  lcd_print("Ontozorendszer");
  delay_ms(2000);
  lcd_clear();
}


void loop() {
  uint16_t moistureValue = adc_read(MoistureSensorPin);
  float moisture = map(moistureValue, 0, 1023, 0, 100);

  // Motor settings
  if (moisture < setL_moisture) {
    PORTB |= (1 << motorRelayPin); // Turn on motor
  } else if (moisture > setH_moisture) {
    PORTB &= ~(1 << motorRelayPin); // Turn off motor
  }

  //buttons
  if (!(PINC & (1 << bt_set))) {
    if (!flag) {
      flag = 1;
      Set = (Set + 1) % 3;  // scrolling in the menu
      flash = 0;
      delay_ms(200);
    }
  } else {
    flag = 0;
  }

  if (!(PINC & (1 << bt_up))) {
    if (Set == 1) setL_moisture = fmin(setL_moisture + 1, 100);
    if (Set == 2) setH_moisture = fmin(setH_moisture + 1, 100);
    eeprom_write(10 + Set, (uint8_t)(Set == 1 ? setL_moisture : setH_moisture));
    delay_ms(10);
  }

  if (!(PINC & (1 << bt_down))) {
    if (Set == 1) setL_moisture = fmax(setL_moisture - 1, 0);
    if (Set == 2) setH_moisture = fmax(setH_moisture - 1, 0);
    eeprom_write(10 + Set, (uint8_t)(Set == 1 ? setL_moisture : setH_moisture));
    delay_ms(10);
  }

  // LCD display
  unsigned long currentMillis = millis();
  if (currentMillis - lastLcdUpdate >= lcdUpdateInterval) {
    lastLcdUpdate = currentMillis;

    lcd_clear();
    if (Set == 0) {
      lcd_set_cursor(0, 0);
      lcd_print("Nedvesseg:");
      lcd_set_cursor(11, 0);
		
      //displaying moisture:
      // moisture value ---> string
      char moistureStr[8];
      dtostrf(moisture, 5, 1, moistureStr); //float --> string
      lcd_print(moistureStr);
	  USART_string((unsigned char *)"Nedvesseg:");
      USART_string((unsigned char *)moistureStr);
      USART_string((unsigned char *)"\n\r");
      

      
      //displaying motor statuses:
      lcd_set_cursor(0, 1);
      lcd_print("Motor:");
      lcd_set_cursor(7, 1);
      USART_string((unsigned char *)"Motor:");
      USART_string((unsigned char *)((PORTB & (1 << motorRelayPin)) ? "be" : "ki"));
      USART_string((unsigned char *)"\n\r");
      lcd_print((PORTB & (1 << motorRelayPin)) ? "be" : "ki");
      (PORTB & (1 << motorRelayPin)) ? PORTB |= (1<<PB3) : PORTB &= ~(1<<PB3);
      

      //displaying menu items:
    } else {
      lcd_set_cursor(0, 0);
      lcd_print("Beallitasok:");
      lcd_set_cursor(0, 1);
      lcd_print("L:");
      lcd_set_cursor(2, 1);
      USART_string((unsigned char *)"Beallitasok: L: ");
      
		
      // min és max moisture --> string
      char setMoistureStr[8];
      if (Set == 1 && flash) {
        lcd_print("   ");
      } else {
        dtostrf(setL_moisture, 5, 1, setMoistureStr);
        lcd_print(setMoistureStr);
        USART_string((unsigned char *)setMoistureStr);
      }

      //HIGH menu:
      lcd_set_cursor(9, 1);
      lcd_print("H:");
      USART_string((unsigned char *)"  H: ");
      lcd_set_cursor(11, 1);

      if (Set == 2 && flash) {
        lcd_print("   ");
      } else {
        dtostrf(setH_moisture, 5, 1, setMoistureStr);
        lcd_print(setMoistureStr);
        USART_string((unsigned char *)setMoistureStr);
        USART_string((unsigned char *)"\n\r");
      }
    }
    flash = !flash;
  }
}


//serial port initialization
void USART_Init(unsigned int ubrr)
{
	/* baud rate settings */
  UBRR0H = (unsigned char)(ubrr>>8); //16 bits upper byte is shifted down to lower byte and then pass
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Frame format setting: 8 data, 2stop bit */
  UCSR0C = (1<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);
}


//usart transmit
void USART_Transmit(unsigned char data)
{
  /* Wait for empty transmit buffer */
  while (!(UCSR0A & (1<<UDRE0)));
  /* Put data into buffer, sends the data */
  UDR0 = data;
}


//usart receive
unsigned char USART_Receive(void)
{
/* wait to receive the data */
while (!(UCSR0A & (1<<RXC0)))
;
/* retrieve and return data from the buffer */
return UDR0;
}

void USART_string(unsigned char *s)
{
	while(*s) //s*: value at the given memory address
    {
    	USART_Transmit(*s++);
    }
}

//LED
void led_init(void)
{
	DDRB |= (1<<PB5);
  	DDRB |= (1<<PB3);
}

//Function Definitions
void lcd_init() {
  lcd_command(0x33); // Init.  4-bit mode
  lcd_command(0x32);
  lcd_command(0x28); // 2 line, 5x8 font
  lcd_command(0x0C); // Display ON, Cursor OFF
  lcd_command(0x06); // Auto increment
  lcd_command(0x01); // Clear display
  delay_ms(2);
}

void lcd_command(uint8_t cmd) {
  PORTD = (PORTD & 0x0F) | (cmd & 0xF0); // Send high nibble
  PORTD &= ~(1 << LCD_RS);
  PORTD |= (1 << LCD_EN);
  delay_ms(1);
  PORTD &= ~(1 << LCD_EN);

  PORTD = (PORTD & 0x0F) | ((cmd << 4) & 0xF0); // Send low nibble
  PORTD |= (1 << LCD_EN);
  delay_ms(1);
  PORTD &= ~(1 << LCD_EN);
}

void lcd_data(uint8_t data) {
  PORTD = (PORTD & 0x0F) | (data & 0xF0); // Send high nibble
  PORTD |= (1 << LCD_RS);
  PORTD |= (1 << LCD_EN);
  delay_ms(1);
  PORTD &= ~(1 << LCD_EN);

  PORTD = (PORTD & 0x0F) | ((data << 4) & 0xF0); // Send low nibble
  PORTD |= (1 << LCD_EN);
  delay_ms(1);
  PORTD &= ~(1 << LCD_EN);
}

void lcd_print(const char* str) {
  while (*str) lcd_data(*str++);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  uint8_t address = (row == 0 ? 0x80 : 0xC0) + col;
  lcd_command(address);
}

void lcd_clear() {
  lcd_command(0x01); //clear display
  delay_ms(2);
}

uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
  ADCSRA |= (1 << ADSC); //start conversion
  while (ADCSRA & (1 << ADSC)); //wait for conversion to finish
  return ADC;
}

void eeprom_write(uint8_t address, uint8_t value) {
  while (EECR & (1 << EEPE)); //wait for previous write to finish
  EEAR = address;
  EEDR = value;
  EECR = (1 << EEMPE); // Enable master write
  EECR |= (1 << EEPE); //start writing
}

uint8_t eeprom_read(uint8_t address) {
  while (EECR & (1 << EEPE)); // wainting
  EEAR = address;
  EECR |= (1 << EERE); //start reading
  return EEDR;
}

//custom delay_ms function: takes value from variable
void delay_ms(uint16_t ms) {
  while (ms--) _delay_ms(1); 
}

//TIMER2 inic.
void timer_init(void)
{
  //MODE
	//-----Normal
	//				WGM2 WGM1 WGM0: 0 0 0	Normal
	//-----CTC
	//				WGM2 WGM1 WGM0:	0 1 0 	CTC
	//PRESCALE
	//-----1024
	//				CS22 CS21 CS20: 1 1 1 	clkT2S/1024
	//-----64
	//				CS22 CS21 CS20: 1 0 0	clkT2S/64
	//EN IT
	//-----OVF
	//				TOIE2: 1				OVF
	//-----OC
	//				OCIE2A: 1				OC_A
	//GLOBAL IT EN
	//	COM2A1 COM2A0 COM2B1 COM2B0 – – WGM21 WGM20 	|TCCR2A|
	// 	FOC2A FOC2B – – WGM22 CS22 CS21 CS20 			|TCCR2B|
	//	– – – – – OCIE2B OCIE2A TOIE2 					|TIMSK2|
	
	//Normal, OVF, 1024
	TCCR2A = (0<<WGM21) | (0<<WGM20);
	TCCR2B = (0<<WGM22)	| (1<<CS22) | (1<<CS21) | (1<<CS20);
	TIMSK2 = (1<<TOIE2);
	

	//CTC, OC_A, 64, Comp: 250-1
/*
	TCCR2A = (1<<WGM21) | (0<<WGM20);
	TCCR2B = (0<<WGM22)	| (1<<CS22) | (0<<CS21) | (0<<CS20);
	OCR2A = 250-1;
	TIMSK2 = (1<<OCIE2A);
*/
	sei(); //enables global interrupt
}
