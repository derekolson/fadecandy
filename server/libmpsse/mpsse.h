#ifndef _LIBMPSSE_H_ 
#define _LIBMPSSE_H_

#include <stdint.h>

#include <ftdi.h>

#define MPSSE_OK		0
#define MPSSE_FAIL		-1

#define MSB			0x00
#define LSB			0x08

#define CHUNK_SIZE		65535
#define SPI_RW_SIZE		(63 * 1024) 
#define SPI_TRANSFER_SIZE	512
#define I2C_TRANSFER_SIZE	64

#define LATENCY_MS		2
#define TIMEOUT_DIVISOR		1000000
#define USB_TIMEOUT		120000
#define SETUP_DELAY		25000

#define BITMODE_RESET		0
#define BITMODE_MPSSE		2

#define CMD_SIZE		3
#define MAX_SETUP_COMMANDS	10
#define SS_TX_COUNT		3

#define LOW			0
#define HIGH			1
#define NUM_GPIOL_PINS		4
#define NUM_GPIO_PINS		12

#define NULL_CONTEXT_ERROR_MSG	"NULL MPSSE context pointer!"

/* FTDI interfaces */
enum interface
{
	IFACE_ANY	= INTERFACE_ANY,
	IFACE_A 	= INTERFACE_A,
	IFACE_B		= INTERFACE_B,
	IFACE_C		= INTERFACE_C,
	IFACE_D		= INTERFACE_D
};

/* Common clock rates */
enum clock_rates
{
	ONE_HUNDRED_KHZ  = 100000,
	FOUR_HUNDRED_KHZ = 400000,
	ONE_MHZ 	 = 1000000,
	TWO_MHZ		 = 2000000,
	FIVE_MHZ	 = 5000000,
	SIX_MHZ 	 = 6000000,
	TEN_MHZ		 = 10000000,
	TWELVE_MHZ 	 = 12000000,
	FIFTEEN_MHZ      = 15000000,
	THIRTY_MHZ 	 = 30000000,
	SIXTY_MHZ 	 = 60000000
};

/* Supported MPSSE modes */
enum modes
{
	SPI0    = 1,
	SPI1    = 2,
	SPI2    = 3,
	SPI3    = 4,
	I2C     = 5,
	GPIO    = 6,
	BITBANG = 7,
};

enum pins
{
	SK	= 1,
	DO	= 2,
	DI	= 4,
	CS	= 8 ,
	GPIO0	= 16,
	GPIO1	= 32,
	GPIO2	= 64,
	GPIO3	= 128
};

enum gpio_pins
{
	GPIOL0 = 0,
	GPIOL1 = 1,
	GPIOL2 = 2,
	GPIOL3 = 3,
	GPIOH0 = 4,
	GPIOH1 = 5,
	GPIOH2 = 6,
	GPIOH3 = 7,
	GPIOH4 = 8,
	GPIOH5 = 9,
	GPIOH6 = 10,
	GPIOH7 = 11
};

enum i2c_ack
{
	ACK  = 0,
	NACK = 1
};

#define DEFAULT_TRIS            (SK | DO | CS | GPIO0 | GPIO1 | GPIO2 | GPIO3)  /* SK/DO/CS and GPIOs are outputs, DI is an input */
#define DEFAULT_PORT            (SK | CS)       				/* SK and CS are high, all others low */

enum mpsse_commands
{
	INVALID_COMMAND		= 0xAB,
	ENABLE_ADAPTIVE_CLOCK   = 0x96,
	DISABLE_ADAPTIVE_CLOCK  = 0x97,
	ENABLE_3_PHASE_CLOCK	= 0x8C,
	DISABLE_3_PHASE_CLOCK	= 0x8D,
	TCK_X5			= 0x8A,
	TCK_D5			= 0x8B,
	CLOCK_N_CYCLES		= 0x8E,
	CLOCK_N8_CYCLES		= 0x8F,
	PULSE_CLOCK_IO_HIGH	= 0x94,
	PULSE_CLOCK_IO_LOW	= 0x95,
	CLOCK_N8_CYCLES_IO_HIGH	= 0x9C,
	CLOCK_N8_CYCLES_IO_LOW	= 0x9D,
	TRISTATE_IO		= 0x9E,
};

enum low_bits_status
{
	STARTED,
	STOPPED
};

struct vid_pid
{
	int vid;
	int pid;
	char *description;
};

struct mpsse_context
{
	char *description;
	struct ftdi_context ftdi;
	enum modes mode;
	enum low_bits_status status;
	int flush_after_read;
	int vid;
	int pid;
	int clock;
	int xsize;
	int open;
	int endianess;
	uint8_t tris;
	uint8_t pstart;
	uint8_t pstop;
	uint8_t pidle;
	uint8_t gpioh;
	uint8_t trish;
	uint8_t bitbang;
	uint8_t tx;
	uint8_t rx;
	uint8_t txrx;
	uint8_t tack;
	uint8_t rack;
};

#ifdef __cplusplus  
extern "C" {
#endif

struct mpsse_context *mpsse_init(enum modes mode, int freq, int endianess);
struct mpsse_context *mpsse_open(int vid, int pid, enum modes mode, int freq, int endianess, int interface, const char *description, const char *serial);
struct mpsse_context *mpsse_open_index(int vid, int pid, enum modes mode, int freq, int endianess, int interface, const char *description, const char *serial, int index);
struct mpsse_context *mpsse_open_dev(struct libusb_device *dev, int freq);
void mpsse_close(struct mpsse_context *mpsse);
const char *mpsse_error_str(struct mpsse_context *mpsse);
int mpsse_set_mode(struct mpsse_context *mpsse, int endianess);
void mpsse_enable_bitmode(struct mpsse_context *mpsse, int tf);
int mpsse_set_clock(struct mpsse_context *mpsse, uint32_t freq);
int mpsse_get_clock(struct mpsse_context *mpsse);
int mpsse_get_vid(struct mpsse_context *mpsse);
int mpsse_get_pid(struct mpsse_context *mpsse);
const char *mpsse_get_desc(struct mpsse_context *mpsse);
int mpsse_set_loopback(struct mpsse_context *mpsse, int enable);
void mpsse_cs_idle(struct mpsse_context *mpsse, int idle);
int mpsse_start(struct mpsse_context *mpsse);
int mpsse_write(struct mpsse_context *mpsse, char *data, int size);
int mpsse_stop(struct mpsse_context *mpsse);
int mpsse_get_ack(struct mpsse_context *mpsse);
void mpsse_set_ack(struct mpsse_context *mpsse, int ack);
void mpsse_send_acks(struct mpsse_context *mpsse);
void mpsse_send_nacks(struct mpsse_context *mpsse);
void mpsse_flush_after_read(struct mpsse_context *mpsse, int tf);
int mpsse_pin_high(struct mpsse_context *mpsse, int pin);
int mpsse_pin_low(struct mpsse_context *mpsse, int pin);
int mpsse_set_direction(struct mpsse_context *mpsse, uint8_t direction);
int mpsse_write_bits(struct mpsse_context *mpsse, char bits, int size);
char mpsse_read_bits(struct mpsse_context *mpsse, int size);
int mpsse_write_pins(struct mpsse_context *mpsse, uint8_t data);
int mpsse_read_pins(struct mpsse_context *mpsse);
int mpsse_pin_state(struct mpsse_context *mpsse, int pin, int state);
int mpsse_tristate(struct mpsse_context *mpsse);
char mpsse_version(void);

char *mpsse_read(struct mpsse_context *mpsse, int size);
char *mpsse_transfer(struct mpsse_context *mpsse, char *data, int size);

static unsigned char mpsse_fast_rw_buf[SPI_RW_SIZE + CMD_SIZE];
int mpsse_fast_write(struct mpsse_context *mpsse, unsigned char *data, int size);
int mpsse_fast_read(struct mpsse_context *mpsse, char *data, int size);
int mpsse_fast_transfer(struct mpsse_context *mpsse, char *wdata, char *rdata, int size);

#ifdef __cplusplus  
}
#endif

#endif
