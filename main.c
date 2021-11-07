#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdbool.h>

#include "def.h"
#include "usb.h"

//----------------------------------------------------------------------
// IR receiver definitions
//----------------------------------------------------------------------
#define MINBIT			10
#define MAXBIT			30
// Timeout for preamble, (9+4.5)ms
#define MINPREAMBLE1	100
#define MAXPREAMBLE1	110
#define MINPREAMBLE2	50
#define MAXPREAMBLE2	55
// 110 ms repeat
#define MAXREPEAT		1300

#define	IR_MAX			5	// maximum number of IR data bytes

#define	LED				(D,5)	// I/O port for LED
#define	SENSOR			(D,6)	// I/O port for IR sensor

#define	NEC_ID_BYTE		0xFC

//----------------------------------------------------------------------
// decoder states
typedef enum
{
	S_IDLE,
	S_PREAMBLE1,
	S_PREAMBLE2,
	S_ADDRESS_0,
	S_ADDRESS_1,
	S_CODE_0,
	S_CODE_1,
	S_LAST
} NecState;

// IgorPlug definitions
enum
{
	USBTINY_ECHO,
	// IgorPlug-USB requests
	IGORPLUG_CLEAR,		// clear IR data
	IGORPLUG_READ		// read IR data (wValue: offset)
};

typedef union
{
	struct			// IgorPlug-USB compatible data layout
	{
		byte_t length;			// length of data[]
		byte_t count;			// incremented for each IR packet
		byte_t offset;			// not used
		byte_t data[IR_MAX];	// decoded data
	};
	byte_t raw[IR_MAX + 3];
} IrPacket;

//----------------------------------------------------------------------
static IrPacket ir;// = {.length = 0, .count = 0, .offset = 0, .data = {NEC_ID_BYTE, 0, 0, 0, 0}};
static byte_t bitCnt;
static byte_t currByte;

//volatile Flags flags;
volatile bool waitRepeat;
volatile NecState state;
volatile byte_t inpos = 0xff;	// read position for usb_in(), or 0xff

//----------------------------------------------------------------------
// noise canceler, trigger on negative edge, clock source clk/1024
#define ENABLE_TCCR1()	do { TCCR1B = _BV(ICNC1) | _BV(CS12) | _BV(CS10); } while(0)
#define DISABLE_TCCR1() do { TCCR1B = 0; } while(0)

//----------------------------------------------------------------------
// Handler for timer1 input capture interrupt: edge on IR input
//----------------------------------------------------------------------
ISR(TIMER1_CAPT_vect)
{
	uint8_t stamp = ICR1L;	// get time stamp
	TIMSK = 0;	// disable IR interrupts
	sei();		// allow USB interrupt

	switch(state)
	{
		case S_IDLE:
			BIT_SET(TIFR, OCF1A);
			OCR1AL = MAXPREAMBLE1;
			state = S_PREAMBLE1;
			break;

		case S_PREAMBLE1:
			if(stamp < MINPREAMBLE1)
				state = S_IDLE;
			else
			{
				state = S_PREAMBLE2;
				OCR1AL = MAXPREAMBLE2;
			}
			break;

		case S_PREAMBLE2:
			if(stamp < MINPREAMBLE2)
			{
				state = S_IDLE;
				if(waitRepeat && (stamp >= (MINPREAMBLE2 / 2)))
				{
					//repeat
					++ir.count;
				}
				else
				{
					// cancel packet
					ir.length = 0;
					break;
				}
			}
			else
			{
				// prepare for new packet
				state = S_ADDRESS_0;
				currByte = 0;
				bitCnt = 0;
				ir.length = 1;
			}

			SET(LED);	// switch LED on
			BIT_SET(TIFR, OCF1B);	// clear Output Compare Interrupt 1B
			OCR1B = MAXREPEAT;
			OCR1AL = MAXBIT;
			break;

		default:	// data bits
			if(stamp < MINBIT)
				state = S_IDLE;
			else
			{
				currByte <<= 1;
				if(stamp > MAXBIT/2)
					currByte |= 1;
				++bitCnt;

				if(bitCnt == 8)
				{
					cli();
					if(inpos == 0xFF)
						ir.data[ir.length++] = currByte;
					sei();

					if(++state < S_LAST)
					{
						bitCnt = 0;
						currByte = 0;
					}
					else
					{
						++ir.count;		// пакет закончен
						waitRepeat = 1;
						state = S_IDLE;
					}
				}
			}
	}

	if(state < S_ADDRESS_0)		// counting only NEC edges
		TCCR1B ^= _BV(ICES1);	// toggle edge detector
	TCNT1 = 0;
	cli();
	TIMSK = ir.length? _BV(ICIE1) | _BV(OCIE1A) | _BV(OCIE1B) : _BV(ICIE1) | _BV(OCIE1A);
}

// ----------------------------------------------------------------------
// Handler for timer1 output compare A interrupt: IR transmission timeout
// ----------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
	TIMSK = 0;
	sei();

	state = S_IDLE;
	ENABLE_TCCR1();		// reset to negative edge
	CLR(LED);			// switch LED off

	cli();
	TIMSK = waitRepeat? _BV(ICIE1) | _BV(OCIE1B) : _BV(ICIE1);
}

// ----------------------------------------------------------------------
// Handler for timer1 output compare B interrupt: repeat code timeout
// ----------------------------------------------------------------------
ISR(TIMER1_COMPB_vect)
{
	waitRepeat = 0;
	TIMSK = _BV(ICIE1);
}

// ----------------------------------------------------------------------
// Handle a non-standard SETUP packet.
// ----------------------------------------------------------------------
extern byte_t usb_setup(byte_t data[8])
{
	byte_t r = 0;
	switch(data[1])
	{
	// IgorPlug-USB requests
		case IGORPLUG_CLEAR:
			inpos = 0xff;
			break;

		case IGORPLUG_READ:
			cli();
			if(ir.length == IR_MAX)
			{
				inpos = data[2];
				r = 0xFF;	// call usb_in() to get the data
			}
			else
			{
				data[0] = 0;
				r = 1;
			}
			sei();
			break;
	}

	return r;
}

// ----------------------------------------------------------------------
// Handle an IN packet.
// ----------------------------------------------------------------------
extern byte_t usb_in(byte_t* data, byte_t len)
{
	byte_t n = 0;
	while(n < len && inpos < (IR_MAX + 3))
		data[n++] = ir.raw[inpos++];

	inpos = 0xff;	// reenable receiver
	return n;
}

// ----------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------
extern int main(void)
{
	// Initialize the IR receiver.
	OUTPUT(LED);
	SET(SENSOR);	//pullup

	ir.data[0] = NEC_ID_BYTE;

	ENABLE_TCCR1();
	TIMSK = _BV(ICIE1);	// input capture 1 interrupt enable

	usb_init();
	for(;;)
	{
		usb_poll();
	}
}
