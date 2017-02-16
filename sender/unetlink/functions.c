#include "functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <fftw3.h>
#include <sys/mman.h>
#include <time.h>


long OFDM_readFile(char* fileName, uint8_t** buffer)
{
	*buffer = NULL;
	long length;
	int fd = open(fileName, O_RDONLY);

	if (fd)
	{
		length = (long)lseek (fd, 0, SEEK_END);
		lseek (fd, 0, SEEK_SET);
		*buffer = (uint8_t*) malloc (length);
		
		if (*buffer)
		{
			read (fd, *buffer, length);
			//printf("reading\n");
		}
		else
		{
			close (fd);
			return OFDM_AllocationError;
		}
		close (fd);
	}
	else
	{
		return OFDM_FileError;
	}
	//printf("buffer is %02x %02x %d",(*buffer)[0],(*buffer)[length-1],length);
	return length;

}


void modulate(uint8_t* data,long Length, long offset, fftw_complex* out)
{
	uint8_t* in = NULL;
	int i = 0;
	int j = 0;
	int k = 0;
	uint8_t mask= 0x80;
	//printf("modulating\n");
	in = (uint8_t*)malloc (Length*8*sizeof(uint8_t));
	if(in == NULL)
	{
		printf("it's null don't go further");
	}
	
	for(k = 0; k < 64; k++)
	{
		out[k][0] = 0;
		out[k][1] = 0;
	}
	for(i = 0; i<Length; i++)
	{
		//printf("the byte is %02x\n",data[offset+i]);
		for(j = 0; j < 8; j++)
		{
			//printf("mask is %02x\n",mask);
			//printf("i is %d, j is %d\n",i,j);
			in[i*8+j] =(uint8_t) ((data[offset+i] & mask) >> (7-j));
			out[i*8+j+1][0] = bpsk_i[in[i*8+j]];
			out[i*8+j+1][1] = 0;
			out[SIZE_IFFT-(i*8+j)-1][0] = bpsk_i[in[i*8+j]];
			out[SIZE_IFFT-(i*8+j)-1][1] = 0;
			//printf("%x",in[i*8+j]);
			//printf("shifting\n");
			mask = mask >> 0x1 ;
			
		}
		mask= 0x80;
		//printf("\n");
		//printf("next Byte please\n");
	}
	
	//printf("done\n");
	free(in);
	
	
	
}

void performIfft(fftw_complex* mod, fftw_complex* time_out)
{
	fftw_plan ifft;

	fftw_complex *in = fftw_alloc_complex(64);
	fftw_complex *out = fftw_alloc_complex(64);

	int i;
	for (i = 0; i < 64; i++) {
		in[i][0] = mod[i][0];
		in[i][1] = mod[i][1];
	}

	ifft = fftw_plan_dft_1d(64, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
	fftw_execute(ifft);

	for (i = 0; i < 64; i++) {
		time_out[i][0] = out[i][0];
		time_out[i][1] = out[i][1];
	}

	fftw_destroy_plan(ifft);

	fftw_free(in);
	fftw_free(out);
}

void initGuard(fftw_complex* guard)
{
	guard[0][0] = 0;
	guard[1][0] = 32;
	guard[2][0] = 64;
	guard[3][0] = 32;
	guard[4][0] = 0;
	guard[5][0] = -32;
	guard[6][0] = -64;
	guard[7][0] = -32;
	guard[8][0] = 0;
	guard[9][0] = 0;
	guard[10][0] = 0;
	guard[11][0] = 0;
	guard[12][0] = 0;
	guard[13][0] = 0;
	guard[14][0] = 0;
	guard[15][0] = 32;
	guard[16][0] = 64;
	guard[17][0] = 32;
	guard[18][0] = 0;
	guard[19][0] = -32;
	guard[20][0] = -64;
	guard[21][0] = -32;
	guard[22][0] = 0;
	
}

void sum_samples(fftw_complex *in_a, fftw_complex *in_b, int size_b, int base_index) {

	int i;

	for (i = 0; i < size_b; i++) {

		in_a[i + base_index][0] += in_b[i][0];
		in_a[i + base_index][1] += in_b[i][1];
		printf("sample $d is %lf\n", i + base_index, in_a[i + base_index][0]);
	}

}

void convert_signal_to_levels(fftw_complex* signal, fftw_complex* levels, int max, int size)
{
	int i=0;
	for(i=0;i<size;i++)
	{
		levels[i][0]=(signal[i][0]*5+2357);
		printf("level %d : %lf \n", i, levels[i][0]);
	}

}

int init_gpios()
{
	fdm = -1;
	
/*	gpio1_addr = NULL;
	gpio1_oe_addr = NULL;
	gpio1_setdataout_addr = NULL;
	gpio1_cleardataout_addr = NULL;

	gpio2_addr = NULL;
	gpio2_oe_addr = NULL;
	gpio2_setdataout_addr = NULL;
	gpio2_cleardataout_addr = NULL;

	gpio3_addr = NULL;
	gpio3_oe_addr = NULL;
	gpio3_setdataout_addr = NULL;
	gpio3_cleardataout_addr = NULL;
*/	


	pin_map[0].gpio = 3;
	pin_map[0].number = 1 << 20;
//	printf("gpio pin of 0 is %d", pin_map[0].gpio);
	pin_map[1].gpio = 2;
	pin_map[1].number = 1 << 2;
	pin_map[2].gpio = 2;
	pin_map[2].number = 1 << 3;
	pin_map[3].gpio = 2;
	pin_map[3].number = 1 << 5;
	pin_map[4].gpio = 2;
	pin_map[4].number = 1 << 4;
	pin_map[5].gpio = 3;
	pin_map[5].number = 1 << 16;
	pin_map[6].gpio = 3;
	pin_map[6].number = 1 << 19;
	pin_map[7].gpio = 3;
	pin_map[7].number = 1 << 21;
	pin_map[8].gpio = 1;
	pin_map[8].number = 1 << 17;
	pin_map[9].gpio = 1;
	pin_map[9].number = 1 << 16;
	pin_map[10].gpio = 1;
	pin_map[10].number = 1 << 28;
	
	clc.gpio=1;
	clc.number=1 << 13;
	miso.gpio=0;
	miso.number=1 << 23;
	mosi.gpio=1;
	mosi.number=1 << 15;
	cs.gpio=1;
	cs.number=1 << 12;

	printf("gpio pin of 10 is %d", pin_map[10].gpio);

	fdm = open("/dev/mem", O_RDWR);
	
	if(fdm < 0)
	{
		return OFDM_GeneralError;
	}
    /*printf("Mapping %X - %X (size: %X)\n", GPIO1_START_ADDR,
		   GPIO1_END_ADDR, GPIO1_SIZE);*/

//	printf("gpio1 init\n");
	
	// GPIO1 initialisation
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
void nano_sleep()
{
   struct timespec tim, tim2;
   tim.tv_sec = 1;
   tim.tv_nsec = 1000;

   if(nanosleep(&tim , &tim2) < 0 )   
   {
      printf("Nano sleep system call failed \n");
      return -1;
   }

}
void SPI_write(uint16_t data)
{
    uint16_t shift = 0x8000; 
    while (shift > 0) {
        //writel(bit_clc, gpio2+CLEAR_OFFSET);
        clear_gpio(clc);
//	nano_sleep();
        delay_n_NOP();
        if ((data & shift) != 0) {
            //writel(1<<BIT_MOSI, gpio2+SET_OFFSET);
//		printf("1\n");
            set_gpio(mosi);
            delay_n_NOP();
        } else {
            //writel(1<<BIT_MOSI, gpio2+CLEAR_OFFSET);
            clear_gpio(mosi);
//		printf("0\n");
            delay_n_NOP();
        }
        shift >>= 1;
        //writel(bit_clc, gpio2+SET_OFFSET);
        set_gpio(clc);
//	nano_sleep();
//        delay_n_NOP();
    }
}

void delay_n_NOP(void)
{
    int i;
    for(i=SPI_DELAY_CNT; i>0; i--) 
        ;
}
