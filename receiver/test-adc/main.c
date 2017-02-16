//#include "functions.h"
#include <stdio.h>
#include <stdlib.h>
//#include <fftw3.h>
#include <unistd.h>
#include <string.h> 
#include <time.h> 
#include <signal.h> 
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdint.h>

//fftw_complex *levels;
struct itimerval it;
typedef enum
{
        OFDM_GeneralError = -1,
        OFDM_FileError = -2,
        OFDM_AllocationError = -3
}OFDM_errors;

#define GPIO1_START_ADDR 0x4804C000
#define GPIO1_END_ADDR 0x4804DFFF
#define GPIO2_START_ADDR 0x481AC000
#define GPIO3_START_ADDR 0x481AE000
#define GPIO1_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)
#define GPIO_OE 0x134
#define GPIO_SETDATAOUT 0x194
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_READDATA 0x138

struct pin
{
        int gpio;
        uint32_t number;
};


struct pin clc;
struct pin miso;
struct pin mosi;
struct pin cs;

int fdm;

void *gpio1_addr;
unsigned int *gpio1_oe_addr;
unsigned int *gpio1_setdataout_addr;
unsigned int *gpio1_cleardataout_addr;
unsigned int *gpio1_readdata_addr;

void *gpio2_addr;
unsigned int *gpio2_oe_addr;
unsigned int *gpio2_setdataout_addr;
unsigned int *gpio2_cleardataout_addr;
unsigned int *gpio2_readdata_addr;

void *gpio3_addr;
unsigned int *gpio3_oe_addr;
unsigned int *gpio3_setdataout_addr;
unsigned int *gpio3_cleardataout_addr;
unsigned int *gpio3_readdata_addr;

int reg1;
int reg2;
int reg3;

void delay_n_NOP(void)
{
    int i;
    for(i=10; i>0; i--) 
        ;
}

int init_gpios()
{
	fdm = -1;
	
	clc.gpio=1;
	clc.number=1 << 13;
	miso.gpio=1;
	miso.number=1 << 14;
	mosi.gpio=1;
	mosi.number=1 << 15;
	cs.gpio=1;
	cs.number=1 << 12;

//	printf("gpio pin of 10 is %d", pin_map[10].gpio);

	fdm = open("/dev/mem", O_RDWR);
	
	if(fdm < 0)
	{
		return OFDM_GeneralError;
	}

    gpio1_addr = mmap(0, GPIO1_SIZE, PROT_READ | PROT_WRITE,
					 MAP_SHARED, fdm, GPIO1_START_ADDR);
	if(gpio1_addr == MAP_FAILED) {
        printf("Unable to map GPIO\n");
        return OFDM_GeneralError;
    	}
    printf("gpio 1 address %04x \n", gpio1_addr);

    gpio1_oe_addr = gpio1_addr + GPIO_OE;
    gpio1_setdataout_addr = gpio1_addr + GPIO_SETDATAOUT;
    gpio1_cleardataout_addr = gpio1_addr + GPIO_CLEARDATAOUT;
    gpio1_readdata_addr = gpio1_addr + GPIO_READDATA;

	
	// GPIO2 initialisation
    gpio2_addr = mmap(0, GPIO1_SIZE, PROT_READ | PROT_WRITE,
					 MAP_SHARED, fdm, GPIO2_START_ADDR);

    if(gpio2_addr == MAP_FAILED) {
        printf("Unable to map GPIO\n");
        return OFDM_GeneralError;
    }
    printf("gpio 2 address %04x \n", gpio2_addr);

    gpio2_oe_addr = gpio2_addr + GPIO_OE;
    gpio2_setdataout_addr = gpio2_addr + GPIO_SETDATAOUT;
    gpio2_cleardataout_addr = gpio2_addr + GPIO_CLEARDATAOUT;
    gpio2_readdata_addr = gpio2_addr + GPIO_READDATA;


	
	// GPIO3 initialisation
    gpio3_addr = mmap(0, GPIO1_SIZE, PROT_READ | PROT_WRITE,
					 MAP_SHARED, fdm, GPIO3_START_ADDR);

    if(gpio3_addr == MAP_FAILED) {
        printf("Unable to map GPIO\n");
        return OFDM_GeneralError;
    }
    printf("gpio 3 address %04x", gpio3_addr);
    gpio3_oe_addr = gpio3_addr + GPIO_OE;
    gpio3_setdataout_addr = gpio3_addr + GPIO_SETDATAOUT;
    gpio3_cleardataout_addr = gpio3_addr + GPIO_CLEARDATAOUT;
    gpio3_readdata_addr = gpio3_addr + GPIO_READDATA;


	printf("init done \n");
	
	return 0;

}

void set_gpio(struct pin p)
{
//	printf("setting gpio %d,%04x\n",p.gpio,p.number);
	switch(p.gpio)
	{
		case 1:
		{
			reg1 = *gpio1_oe_addr;
			reg1 = reg1 & (0xFFFFFFFF - p.number);
			*gpio1_oe_addr = reg1;
			*gpio1_setdataout_addr = p.number;
			break;
		}
		case 2:
		{
			//printf("please print this");
			reg2 = *gpio2_oe_addr;
			reg2 = reg2 & (0xFFFFFFFF - p.number);
			*gpio2_oe_addr = reg2;
			*gpio2_setdataout_addr = p.number;
			break;
		}
		case 3:
		{
			reg3 = *gpio3_oe_addr;
			reg3 = reg3 & (0xFFFFFFFF - p.number);
			*gpio3_oe_addr = reg3;
			*gpio3_setdataout_addr = p.number;
			break;
		}
		default:
		break;
	}
}

void clear_gpio(struct pin p)
{
	switch(p.gpio)
	{
		case 1:
		{
			reg1 = *gpio1_oe_addr;
			reg1 = reg1 & (0xFFFFFFFF - p.number);
			*gpio1_oe_addr = reg1;
			*gpio1_cleardataout_addr = p.number;
			break;
		}
		case 2:
		{
			//printf("clearing please");
			reg2 = *gpio2_oe_addr;
			reg2 = reg2 & (0xFFFFFFFF - p.number);
			*gpio2_oe_addr = reg2;
			*gpio2_cleardataout_addr = p.number;
			break;
		}
		case 3:
		{
			reg3 = *gpio3_oe_addr;
			reg3 = reg3 & (0xFFFFFFFF - p.number);
			*gpio3_oe_addr = reg3;
			*gpio3_cleardataout_addr = p.number;
			break;
		}
		default:
			break;
	}
}

int read_gpio(struct pin p)
{
	switch(p.gpio)
	{
		case 1:
		{
			reg1 = *gpio1_oe_addr;
			reg1 = reg1 & (0xFFFFFFFF - p.number);
			*gpio1_oe_addr = reg1;
			return *gpio1_readdata_addr;
			break;
		}
		case 2:
		{
			//printf("clearing please");
			reg2 = *gpio2_oe_addr;
			reg2 = reg2 & (0xFFFFFFFF - p.number);
			*gpio2_oe_addr = reg2;
			return *gpio2_readdata_addr;
			break;
		}
		case 3:
		{
			reg3 = *gpio3_oe_addr;
			reg3 = reg3 & (0xFFFFFFFF - p.number);
			*gpio3_oe_addr = reg3;
			return *gpio3_readdata_addr;
			break;
		}
		default:
			printf("you should not be here");
			break;
	}
	return -1;
}

void SPI_write_8(uint8_t data)
{
    uint8_t shift = 0x10; 
    while (shift > 0) {
        //writel(bit_clc, gpio2+CLEAR_OFFSET);
        clear_gpio(clc);
//      nano_sleep();
        delay_n_NOP();
        if ((data & shift) != 0) {
            //writel(1<<BIT_MOSI, gpio2+SET_OFFSET);
//              printf("1\n");
            set_gpio(mosi);
            delay_n_NOP();
        } else {
            //writel(1<<BIT_MOSI, gpio2+CLEAR_OFFSET);
            clear_gpio(mosi);
//              printf("0\n");
            delay_n_NOP();
        }
        shift >>= 1;
        //writel(bit_clc, gpio2+SET_OFFSET);
        set_gpio(clc);
//      nano_sleep();
//        delay_n_NOP();
    }
}

void sigalrm_handler(int sig) 
{
	unsigned int value = 0, index;
	//printf("Alarm number %5u at %d\n", ++count, time(0))
	
	// set cs to low
        clear_gpio(cs);
        
        // send high byte
        uint8_t buffer = 0x18;
	//printf("buffer %d\n",buffer);
	//buffer = 0x0FFF & buffer;
	//buffer = 0x3000 | buffer;
	SPI_write_8(buffer);
        
	clear_gpio(clc);
	delay_n_NOP();

        set_gpio(clc);
	delay_n_NOP();
	
	for (index=0; index<11; index++)
    	{
        	//writel(bit_clc, gpio2+CLEAR_OFFSET);
        	clear_gpio(clc);
		delay_n_NOP();

        	value <<= 1;
        	//value |= (0x1 & (readl(gpio1+READ_OFFSET)>>BIT_MISO));
       		value |= (0x1 & ((*gpio1_readdata_addr)>>miso.number));
		//writel(bit_clc, gpio2+SET_OFFSET);
        	printf("%08x\n",*gpio1_readdata_addr);
		set_gpio(clc);
		delay_n_NOP();
    	}
    	//writel(bit_clc, gpio2+CLEAR_OFFSET);
    	
	clear_gpio(clc);
	delay_n_NOP();
       
	 // set cs to low
        set_gpio(cs);
	delay_n_NOP();
	printf("val is : %d\n",value);
}

int main(int argc, char **argv)
{
	int ret = 0;
	/*uint8_t* data = NULL;
	
	long bLength;
	int nSymbols;
	fftw_complex *mod;
	fftw_complex *ifft;
	fftw_complex *Signal;
	fftw_complex *guard;
	struct timespec start_time,stop_time;
	*/
	it.it_value.tv_sec = 1; /* start in 1 second */ 
	it.it_value.tv_usec = 0; 
	it.it_interval.tv_sec = 0; /* repeat every 5 seconds */ 
	it.it_interval.tv_usec = 400;
 
	signal(SIGALRM, sigalrm_handler);
	
	/*mod = fftw_alloc_complex(SIZE_IFFT);
	// read from file
	bLength = OFDM_readFile(argv[1], &data);
	
	if(bLength)
	{
		//printf("file size is %ld \n",bLength);
	}
	else if(bLength == OFDM_FileError)
	{
		printf("file error \n");
		return 0;
	}
	else if(bLength == OFDM_AllocationError)
	{
		printf("allocation error \n");
		return 0;
	}
*/	
	ret = init_gpios();
	if(ret < 0)
	{
		printf("init error");
		return 0;
	}
/*	nSymbols = (((bLength*8)%N_REAL_SUBCARRIERS) == 0 ?((bLength*8)/N_REAL_SUBCARRIERS): ((bLength*8)/N_REAL_SUBCARRIERS) +1 );

	ifft = fftw_alloc_complex(SIZE_IFFT);
	Signal = fftw_alloc_complex((SIZE_IFFT+GUARD_TIME)*nSymbols);
	levels = fftw_alloc_complex((SIZE_IFFT+GUARD_TIME)*nSymbols);
	//printf("number of sybols is %d\n",nSymbols);
	//printf("the first byte is %02x\n",data[100]);
	int i = 0;
	int size;
	
	//generate guard band

	guard = fftw_alloc_complex(GUARD_TIME);

	int max = 128;
	
	//clock_gettime(CLOCK_MONOTONIC, &start_time);
	//start timer
	initGuard(guard);

	for (i=0; i<nSymbols; i++)
	{
		sum_samples(Signal, guard, GUARD_TIME, i*(64+GUARD_TIME));
		size = 0;
		size = ((i+1)*3 <= bLength ? 3 : bLength-i*3);
		//printf("%d:the size is %d\n",i,size);
		modulate(data,size,i*3, mod);
		/*int k = 0;
		for(k = 0 ; k < 64 ; k++)
		{
			printf("%d: mod %lf\n",k, mod[k][0]);

		}*/
		//ifft
		//printf("performing fft\n");
		//performIfft(mod,ifft);
		/*int k = 0;
		for(k = 0 ; k < 64 ; k++)
		{
			printf("%d: real %lf im %lf\n",k, ifft[k][0],ifft[k][1]);
		}*/
		
		
		//sum_samples(Signal, ifft, 64, (i+1)*GUARD_TIME+i*64);
		
		
	//}
	//stop timer
	
//	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	
	//printf("The time took for a buffer of length %d this operation is %ld \n ", bLength, (stop_time.tv_sec - start_time.tv_sec)*1000000000+stop_time.tv_nsec - start_time.tv_nsec);
	
	//int j = 0;
	//double minimum = signal[0][0];
	/*max = 0;
	for(j = 1 ; j < (SIZE_IFFT+GUARD_TIME)* nSymbols; j++)
	{
		//if(minimum > signal[j][0])
		//minimum = signal[j][0];

		if(max < signal[j][0])
		max= signal[j][0];
	}*/
	//printf("maximum is %lf\n",max);
	
	//if(minimum < 0)
	//{
	/*	for(j = 0; j < (SIZE_IFFT+GUARD_TIME)* nSymbols; j++)
		{
			Signal[j][0] += 64;
			printf("%lf\n",Signal[j][0]);
		}
		//max+=64;
		//printf("the max is %lf\n",max);*/
	//}
	
	
	//printf("initializing gpios \n");
	//uint32_t pin = 1 << 7;
    	//convert_signal_to_levels(Signal, levels, max,(SIZE_IFFT+GUARD_TIME)*nSymbols);
	//max = 0;

	setitimer(ITIMER_REAL, &it, NULL);
	
	while(1)
	{}
	/*while(1)
	{

	int s_cpt=0;
     	//for(s_cpt=0;s_cpt<(SIZE_IFFT+GUARD_TIME)*nSymbols;s_cpt++)
    	for(s_cpt=0;s_cpt<(SIZE_IFFT+GUARD_TIME);s_cpt++)
	{
        // set cs to low
        clear_gpio(cs);
        
        // send high byte
        uint16_t buffer = (int)levels[s_cpt][0];
	//printf("buffer %d\n",buffer);
	buffer = 0x0FFF & buffer;
	buffer = 0x3000 | buffer;
	SPI_write(buffer);
        
        // set cs to low
        set_gpio(cs);
	delay_n_NOP();
	usleep(290);
	
	}
	s_cpt = 0;
*/
// test 1

/*	int s_cpt=0;
    	for(s_cpt=1990;s_cpt<2134;s_cpt++)
	{
        // set cs to low
        clear_gpio(cs);
        
        // send high byte
        uint16_t buffer = s_cpt;
	printf("buffer %d\n",buffer);
	buffer = 0x0FFF & buffer;
	buffer = 0x3000 | buffer;
	SPI_write(buffer);
        
        // set cs to low
        set_gpio(cs);
	delay_n_NOP();
	//usleep(200);
	
	}
	s_cpt = 0;
*/

/*
        // set cs to low
        clear_gpio(cs);
        
        // send high byte
        uint16_t buffer = 2106+52 ;
	buffer = 0x0FFF & buffer;
	buffer = 0x3000 | buffer;
	SPI_write(buffer);
        
        // set cs to low
        set_gpio(cs);
	delay_n_NOP();
	//sleep(1);
	

	}
	
/*	while(1)
	{
		set_gpio(pin_map[4]);
		clear_gpio(pin_map[4]);
	}

*/
	//~ set_gpio(pin_map[0]);
	//~ set_gpio(pin_map[1]);
	//~ set_gpio(pin_map[2]);
	//~ set_gpio(pin_map[3]);
	//~ set_gpio(pin_map[4]);
	//~ set_gpio(pin_map[5]);
	//~ set_gpio(pin_map[6]);
	//~ set_gpio(pin_map[7]);
	//~ set_gpio(pin_map[8]);
	//~ set_gpio(pin_map[9]);
//	clear_gpio(pin_map[0]);
/*	clear_gpio(pin_map[2]);
	clear_gpio(pin_map[3]);
	clear_gpio(pin_map[4]);
	clear_gpio(pin_map[5]);
	clear_gpio(pin_map[6]);
	clear_gpio(pin_map[7]);
	clear_gpio(pin_map[8]);
	clear_gpio(pin_map[9]);
	set_gpio(pin_map[10]);
*/	
	/*fftw_free(ifft);
	fftw_free(Signal);
	fftw_free(guard);
	fftw_free(levels);

	fftw_free(mod);
	*/
	return 0;
}
