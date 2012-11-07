#include <avr/io.h>
#include <avr/interrupt.h>

//#define OUTPUT_DSI
#define OUTPUT_PWM
//#define OUTPUT_RELAY

#define DIP2 PB0
#define OUT1 PB1
#define DIP1 PB2
#define RX_LED PB3

#define DIP10 PC0
#define DIP9 PC1
#define DIP8 PC2
#define DIP7 PC3
#define DIP6 PC4
#define DIP5 PC5

#define DMXRX PD0
#define DMXTX PD1
#define DMXTXEN PD2
#define OUT4 PD3
#define DIP4 PD4
#define OUT3 PD5
#define OUT2 PD6
#define DIP3 PD7


#define DMX_BAUD_RATE 250000
#define DMX_MAX_CH 4
#define DMX_MAX_ADDR (511 - DMX_MAX_CH)

uint8_t dmxRxAwaitStart = 1;	// High until Start code received
uint16_t dmxRxPosition;			// Channel count in DMX packet
volatile uint8_t dmxNewData = 0;			// Set when new data received
volatile uint8_t dmxData[DMX_MAX_CH];
uint16_t dmxStartAddress = 0;
uint8_t dmxStartCode = 0;
uint8_t dmxNumCh = 0;
uint8_t dmxRxCount;
uint8_t dmxOffset;
uint8_t dmxMode;

/////////////////////////////////////////////////////////////////////
// DMX handling
//
#ifdef ATMEGA_328P
void dmxRxSetup() {
	// Set DMX to receive mode
	PORTD &= ~_BV(DMXTX);

	// Set up serial port
	UBRR0 = (F_CPU / (DMX_BAUD_RATE * 16L) - 1);
	UCSR0B = _BV(RXCIE0) | _BV(RXEN0);
	UCSR0C = _BV(USBS0) | _BV(UCSZ00) | _BV(UCSZ01);

	// Turn on pullups for DIP switch pins
	PORTC |= _BV(DIP10) | _BV(DIP9) | _BV(DIP8) | _BV(DIP7) | _BV(DIP6) | _BV(DIP5);
	PORTD |= _BV(DIP4) | _BV(DIP3);
	PORTB |= _BV(DIP2) | _BV(DIP1);

	dmxNumCh = DMX_MAX_CH;
	sei();
}
#endif

#ifdef ATMEGA_32u4

void dmxRxSetup() {
	// Set DMX to receive mode
	PORTD &= ~_BV(DMXTX);

	// Set up serial port
	UBRR1 = (F_CPU / (DMX_BAUD_RATE * 16L) - 1);
	UCSR1B = _BV(RXCIE1) | _BV(RXEN1);
	UCSR1C = _BV(USBS1) | _BV(UCSZ10) | _BV(UCSZ11);

	// Turn on pullups for DIP switch pins
	PORTC |= _BV(DIP10) | _BV(DIP9) | _BV(DIP8) | _BV(DIP7) | _BV(DIP6) | _BV(DIP5);
	PORTD |= _BV(DIP4) | _BV(DIP3);
	PORTB |= _BV(DIP2) | _BV(DIP1);

	dmxNumCh = DMX_MAX_CH;
	sei();
}

#endif

// DMX receiver interrupt for Atmega 328p
// Must service within 44us to avoid overflow.
#ifdef ATMEGA_328P

SIGNAL(SIG_USART_RECV) {
	uint8_t uartError, uartData;
	uint8_t *dmxWriteAddr;
	uartError = UCSR0A;
	uartData = UDR0;

	if (uartError & _BV(DOR0)) {
		// Data overrun
		// Important to capture to debug interrupt issues once we get a
		// framework running
	} else if (uartError & _BV(FE0)) {
		// Break condition is start of DMX frame
		dmxRxAwaitStart = 1;
	} else if (dmxRxAwaitStart) {
		dmxRxAwaitStart = 0;
		dmxStartCode = uartData;
		dmxRxPosition = 1;
	} else {
		if (dmxStartCode == 0) {
			// Channel data
			if (dmxRxPosition == dmxStartAddress) {
				dmxRxCount = dmxNumCh;
				dmxWriteAddr = dmxData;
				dmxOffset = 0;
			}
			if (dmxRxCount) {
				dmxData[dmxOffset++] = uartData;
				//*dmxWriteAddr++ = uartData;
				dmxRxCount--;
				if (!dmxRxCount) dmxNewData = 1;
			}
			dmxRxPosition++;
		}
	}
}

void dmxUpdateAddr() {
	uint16_t address = 0;
	if (!(PINB & _BV(DIP1))) address |= 0x001;
	if (!(PINB & _BV(DIP2))) address |= 0x002;
	if (!(PIND & _BV(DIP3))) address |= 0x004;
	if (!(PIND & _BV(DIP4))) address |= 0x008;
	if (!(PINC & _BV(DIP5))) address |= 0x010;
	if (!(PINC & _BV(DIP6))) address |= 0x020;
	if (!(PINC & _BV(DIP7))) address |= 0x040;
	if (!(PINC & _BV(DIP8))) address |= 0x080;
	if (!(PINC & _BV(DIP9))) address |= 0x100;
	dmxStartAddress = address;
	dmxMode = (PINC & _BV(DIP10)) ? 0:1;
	dmxNumCh = 4;
}

#endif


// DMX receiver interrupt for Atmega 32u4
//
#ifdef ATMEGA_32u4

SIGNAL(SIG_USART_RECV) {
	uint8_t uartError, uartData;
	uint8_t *dmxWriteAddr;
	uartError = UCSR1A;
	uartData = UDR1;

	if (uartError & _BV(DOR1)) {
		// Data overrun
		// Important to capture to debug interrupt issues once we get a
		// framework running
	} else if (uartError & _BV(FE1)) {
		// Break condition is start of DMX frame
		dmxRxAwaitStart = 1;
	} else if (dmxRxAwaitStart) {
		dmxRxAwaitStart = 0;
		dmxStartCode = uartData;
		dmxRxPosition = 1;
	} else {
		if (dmxStartCode == 0) {
			// Channel data
			if (dmxRxPosition == dmxStartAddress) {
				dmxRxCount = dmxNumCh;
				dmxWriteAddr = dmxData;
				dmxOffset = 0;
			}
			if (dmxRxCount) {
				dmxData[dmxOffset++] = uartData;
				//*dmxWriteAddr++ = uartData;
				dmxRxCount--;
				if (!dmxRxCount) dmxNewData = 1;
			}
			dmxRxPosition++;
		}
	}
}

void dmxUpdateAddr() {
	uint16_t address = 0;
	if (!(PINB & _BV(DIP1))) address |= 0x001;
	if (!(PINB & _BV(DIP2))) address |= 0x002;
	if (!(PIND & _BV(DIP3))) address |= 0x004;
	if (!(PIND & _BV(DIP4))) address |= 0x008;
	if (!(PINC & _BV(DIP5))) address |= 0x010;
	if (!(PINC & _BV(DIP6))) address |= 0x020;
	if (!(PINC & _BV(DIP7))) address |= 0x040;
	if (!(PINC & _BV(DIP8))) address |= 0x080;
	if (!(PINC & _BV(DIP9))) address |= 0x100;
	dmxStartAddress = address;
	dmxMode = (PINC & _BV(DIP10)) ? 0:1;
	dmxNumCh = 4;
}

#endif

/////////////////////////////////////////////////////////////////////
// DSI output handling
//
#ifdef OUTPUT_DSI

// Clock runs at 16MHz / 64 = 4us ticks
// Counter starts at zero, so value 2 = 3 true counts
#define DSI_PERIOD_COUNTS 207 // 208 counts = 832us

void dsiSetup() {
	// Set up timer 0 for remote transmission
	TCCR0B = 0;				// Freeze clock
	TCCR0A = _BV(WGM01);	// Set in CTC mode
	OCR0A = DSI_PERIOD_COUNTS;
	TCNT0 = 0;				// Zero timer
	TIMSK0 = _BV(OCIE0A);	// Enable interrupt on compare A
	TCCR0B = _BV(CS01) | _BV(CS00); // Clock at 250kHz
}

#define DSI_CHANNELS 4

uint8_t dsiData[DSI_CHANNELS];		// Transmitted value
uint32_t dsiCode[DSI_CHANNELS];		// Manchester encoded version
uint32_t dsiShiftCode[DSI_CHANNELS];// Shifted copy used by interrupt

void dsiUpdate() {
	// Generate DSI bitstreams
	for (uint8_t n=0; n<4; ++n) {
		uint32_t code;
		uint8_t value = dsiData[n];
		code = 1; // Start bit
		uint8_t bits = 8;
		while (bits--) {
			// Manchester encode bit
			code = (code << 2) | ((value & 0x80) ? _BV(1) : _BV(0));
			value <<= 1;
		}
		code <<= 4; // Stop bits
		code <<= 1;
		code |= 1; // Add termination flag for interrupt routine

		// Complete code is 1 start, 16 data periods, 4 stop, 1 terminator = 22 bits
		// Shift left 10 bits to make 32 bit left justified code
		code <<=10;
		dsiCode[n] = code;
	}
}

uint8_t dsiSettleTime = 6;

#define DSI1HIGH() PORTB |= _BV(OUT1)
#define DSI2HIGH() PORTD |= _BV(OUT2)
#define DSI3HIGH() PORTD |= _BV(OUT3)
#define DSI4HIGH() PORTD |= _BV(OUT4)
#define DSI1LOW() PORTB &= ~_BV(OUT1)
#define DSI2LOW() PORTD &= ~_BV(OUT2)
#define DSI3LOW() PORTD &= ~_BV(OUT3)
#define DSI4LOW() PORTD &= ~_BV(OUT4)

SIGNAL(TIMER0_COMPA_vect) {
	if (dsiSettleTime) {
		--dsiSettleTime;
		if (!dsiSettleTime) {
			// Copy codes into shift registers for the following frame
			for (uint8_t n=0; n<4; ++n) dsiShiftCode[n] = dsiCode[n];
		}
	} else {
		uint8_t inactiveCount = 0;
		uint32_t code;

		// This code is unrolled for speed as it is time critical.
		// Otherwise we might miss DMX bytes

		code = dsiShiftCode[0];
		if (code & 0x7fffffff) {
			if (code & 0x80000000) DSI1HIGH(); else DSI1LOW();
			dsiShiftCode[0] <<= 1;
		} else ++inactiveCount;
		
		code = dsiShiftCode[1];
		if (code & 0x7fffffff) {
			if (code & 0x80000000) DSI2HIGH(); else DSI2LOW();
			dsiShiftCode[1] <<= 1;
		} else ++inactiveCount;
		
		code = dsiShiftCode[2];
		if (code & 0x7fffffff) {
			if (code & 0x80000000) DSI3HIGH(); else DSI3LOW();
			dsiShiftCode[2] <<= 1;
		} else ++inactiveCount;
		
		code = dsiShiftCode[3];
		if (code & 0x7fffffff) {
			if (code & 0x80000000) DSI4HIGH(); else DSI4LOW();
			dsiShiftCode[3] <<= 1;
		} else ++inactiveCount;
		
		if (inactiveCount == DSI_CHANNELS) {
			// End of frame reached.
			// Wait 6 bit periods. 833.3us * 6 = 5ms minimum settle time
			dsiSettleTime = 6;
			//DSI1HIGH();
		}
	}
}

void dsiRgb(uint8_t red, uint8_t green, uint8_t blue) {
	dsiData[0] = red;
	dsiData[1] = green;
	dsiData[2] = blue;
	dsiData[3] = 0;
	dsiUpdate();
}
#endif

/////////////////////////////////////////////////////////////////////
// PWM handling
//
#ifdef OUTPUT_PWM
#ifdef ATMEGA_328P
void pwmSet(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
	OCR1A = a;
	OCR0A = b;
	OCR0B = c;
	OCR2B = d;
}


void pwmSetup() {
	pwmSet(0,0,0,0);
	TCCR0A = _BV(WGM00) | _BV(WGM01) | _BV(COM0A1) | _BV(COM0B1);
	TCCR0B = _BV(CS01) | _BV(CS00);
	TCCR1A = _BV(WGM10) | _BV(COM1A1);
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR2A = _BV(WGM20) | _BV(COM2B1);
	TCCR2B = _BV(CS22);
}
#endif
#ifdef ATMEGA_32u4  
void pwmSet(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {		///Work in progress
	OCR1A = a;
	OCR0A = b;
	OCR0B = c;
	OCR2B = d;
}


void pwmSetup() {
	pwmSet(0,0,0,0);
	TCCR0A = _BV(WGM00) | _BV(WGM01) | _BV(COM0A1) | _BV(COM0B1);
	TCCR0B = _BV(CS01) | _BV(CS00);
	TCCR1A = _BV(WGM10) | _BV(COM1A1);
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR2A = _BV(WGM20) | _BV(COM2B1);
	TCCR2B = _BV(CS22);
}
#endif

#endif

/////////////////////////////////////////////////////////////////////
//RELAY output handling
//

#ifdef OUTPUT_RELAY

void relaySet(uint8_t rl1, uint8_t rl2, uint8_t rl3, uint8_t rl4) {
	PORTx |= ((1&(rl1>>7))<<Pxx);
	PORTx |= ((1&(rl2>>7))<<Pxx);
	PORTx |= ((1&(rl3>>7))<<Pxx);
	PORTx |= ((1&(rl4>>7))<<Pxx);
}

void relaySetup(){
	relaySet(0,0,0,0);
}

#endif



/////////////////////////////////////////////////////////////////////
// Core code
//
void setup() {
	// Tweak power reduction register
#ifdef OUTPUT_DSI
#ifdef ATMEGA_328P
	PRR = _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRSPI) | _BV(PRADC);
#elif defined(OUTPUT_PWM)
#ifdef ATMEGA_328P
	PRR = _BV(PRTWI) | _BV(PRSPI) | _BV(PRADC);
#endif
	// Set up ports
	DDRB = _BV(OUT1) | _BV(RX_LED);
	DDRC = 0; // All inputs
	DDRD = _BV(DMXTX) | _BV(DMXTXEN) | _BV(OUT4) | _BV(OUT3) | _BV(OUT2);
	#ifdef OUTPUT_DSI
	dsiSetup();
	#elif defined(OUTPUT_PWM)
	pwmSetup();
	#elif defined(OUTPUT_RELAY)
	relaySetup();
	#endif
	dmxRxSetup();
}

#define DMXTIMEOUT 100000 // About 2 seconds

uint32_t dmxTimeout = DMXTIMEOUT;

uint16_t testCounter;

void loop() {
	dmxUpdateAddr();

	if (dmxStartAddress) {
		testCounter = 0; // Reset test sequence

		if (dmxNewData) {
			dmxTimeout = DMXTIMEOUT;
#ifdef OUTPUT_DSI
			dsiRgb(dmxData[0],dmxData[1],dmxData[2]);
#elif defined(OUTPUT_PWM)
			pwmSet(dmxData[0],dmxData[1],dmxData[2],dmxData[3]);
#elif defined(OUTPUT_RELAY)
			relaySet(dmxData[0],dmxData[1],dmxData[2],dmxData[3]);
#endif
			dmxNewData = 0;
			PORTB |= _BV(RX_LED);
		} else {
			if (dmxTimeout) --dmxTimeout;
			else {
				// No DMX data has been received for a while
#ifdef OUTPUT_DSI
				dsiRgb(0,0,0);
#elif defined(OUTPUT_PWM)
				pwmSet(0,0,0,0);
#elif defined(OUTPUT_RELAY)
				relaySet(0,0,0,0);
#endif
				dmxTimeout = DMXTIMEOUT;
				PORTB &= ~_BV(RX_LED);
			}
		}
	} else {
		uint8_t testBrightness, testChannel;

		// Debug mode when address = 0
		testCounter++;

		// Flicker LED to indicate test mode
		if (testCounter & 0x10) {
			PORTB &= ~_BV(RX_LED);
		} else {
			PORTB |= _BV(RX_LED);
		}

		// Brightness is a ramp up / ramp down signal
		testBrightness = (testCounter & 0xff);
		testBrightness = (testCounter & 0x100) ? ~testBrightness : testBrightness;

		// Channel steps 0, 1, 2, 3 for each ramp
		testChannel = (testCounter >> 9) & 3;

		// Send signal
#ifdef OUTPUT_DSI
		dsiRgb(
			(testChannel == 0) ? testBrightness : 0,
			(testChannel == 1) ? testBrightness : 0,
			(testChannel == 2) ? testBrightness : 0
		);
#elif defined(OUTPUT_PWM)
		pwmSet(
			(testChannel == 0) ? testBrightness : 0,
			(testChannel == 1) ? testBrightness : 0,
			(testChannel == 2) ? testBrightness : 0,
			(testChannel == 3) ? testBrightness : 0
		);
#elif defined(OUTPUT_RELAY)
		relaySet(
			(testChannel == 0) ? testBrightness : 0, // testBrightness doesn't set brightness, channel will stay off until 127 and on from 127 to 255
			(testChannel == 1) ? testBrightness : 0,
			(testChannel == 2) ? testBrightness : 0,
			(testChannel == 3) ? testBrightness : 0
		);
#endif

		// Quick delay loop
		uint32_t delay=2000L; while (delay--) {__asm__("nop\n");}

	}
}

int main() {
	setup();
	dmxNewData = 1;
	dmxData[0] = 0;
	dmxData[1] = 10;
	dmxData[2] = 255;
	while(1) {
		loop();
	}
}
