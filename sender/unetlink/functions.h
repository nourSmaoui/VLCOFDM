#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <stdint.h>
#include <fftw3.h>
/******** ERRORS ************/

typedef enum
{
	OFDM_GeneralError = -1,
	OFDM_FileError = -2,
	OFDM_AllocationError = -3
}OFDM_errors;


/****************** defines *****************/
//number of data subcarriers
#define SIZE_IFFT     64

#define GUARD_TIME    23

#define N_DATA_SUBCARRIERS     48 
//number of real data subcarriers
#define N_REAL_SUBCARRIERS      24
//number of pilot subcarriers
#define N_PILOT_SUBCARRIERS     4

#define GPIO1_START_ADDR 0x4804C000
#define GPIO1_END_ADDR 0x4804DFFF
#define GPIO2_START_ADDR 0x481AC000
#define GPIO3_START_ADDR 0x481AE000
#define GPIO1_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)
#define GPIO_OE 0x134
#define GPIO_SETDATAOUT 0x194
#define GPIO_CLEARDATAOUT 0x190

#define PINT 1<<3

//~ #define BIT_CLC (45-32) // 32+13 P8_11
//~ #define BIT_MISO (23) // 0+23 P8_13
//~ #define BIT_MOSI (47-32) // 32+15 P8_15
//~ //#define BIT_CS (46-32) // 32+14 P8_16
//~ #define BIT_CS (27) // 0+27 P8_17

#define SPI_DELAY_CNT 10


/************************ structs *****************/


struct pin
{
	int gpio;
	uint32_t number;
};





struct pin pin_map[11];
struct pin clc;
struct pin miso;
struct pin mosi;
struct pin cs;

int fdm;

void *gpio1_addr;
unsigned int *gpio1_oe_addr;
unsigned int *gpio1_setdataout_addr;
unsigned int *gpio1_cleardataout_addr;

void *gpio2_addr;
unsigned int *gpio2_oe_addr;
unsigned int *gpio2_setdataout_addr;
unsigned int *gpio2_cleardataout_addr;

void *gpio3_addr;
unsigned int *gpio3_oe_addr;
unsigned int *gpio3_setdataout_addr;
unsigned int *gpio3_cleardataout_addr;

int reg1;
int reg2;
int reg3;

static const double bpsk_i[] = { -1.00, 1.00};
static const double bpsk_q[] = {0, 0};

void set_gpio(struct pin p);

void clear_gpio(struct pin p);

long OFDM_readFile(char* fileName,uint8_t** buffer);

void modulate(uint8_t* data, long Length, long offset, fftw_complex* out);

void performIfft(fftw_complex* mod, fftw_complex* time_out);

void sum_samples(fftw_complex *in_a, fftw_complex *in_b, int size_b, int base_index);

void convert_signal_to_levels(fftw_complex* signal, fftw_complex* levels, int max, int size);

void SPI_write(uint16_t data);

void delay_n_NOP(void);

void initGuard(fftw_complex*);
#endif
