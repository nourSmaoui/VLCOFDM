#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/kthread.h>    // Using kthreads for the flashing functionality
#include <linux/delay.h>      // Using this header for the msleep() function
#include <rtdm/rtdm_driver.h>
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/mm.h>         // mmap related stuff
#include <linux/kfifo.h>      // Using queue to process the producer&consumer model
#include <linux/socket.h>
#include <linux/inet.h>
#include <net/sock.h>



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Trace Yin");
MODULE_DESCRIPTION("ADC driver LKM for the BBB with shared memory between kernel and user");
MODULE_VERSION("0.1");

#define SYMBOL_PERIOD 4000 
//socket
static struct socket *clientsocket=NULL;
#define SERVERPORT 5555
static struct msghdr msg;
static mm_segment_t oldfs;


//direct access with SPI software defined GPIO
volatile void * gpio1;
volatile void * gpio2;
#define ADDR_BASE_0 0x44e07000
#define ADDR_BASE_1 0x4804c000
#define BIT_CLC ( 45-32 )  //32+13  P8_11
#define BIT_MISO  (23)   // 0+23 P8_13
#define BIT_MOSI  (47-32)  // 32+15 P8_15
#define BIT_CS (27)     //0+27   P8_17
#define READ_OFFSET 0x138
#define CLEAR_OFFSET 0x190
#define SET_OFFSET  0x194
#define SPI_CLC  45
#define SPI_MISO  23
#define SPI_MOSI  47
#define SPI_CS  27
#define SPI_DELAY_CNT 10


//gpio FOR debugging LED
static unsigned int gpioLED = 49;           ///< Default GPIO for the LED is 49
module_param(gpioLED, uint, S_IRUGO);       ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioLED, " GPIO LED number (default=49)");     ///< parameter description


//xenomai realtime timer initiator
uint32_t  slot_ns;
static rtdm_timer_t phy_timer;
//static char ledName[7] = "ledXXX";          ///< Null terminated default string -- just in case
static bool ledOn = 0;                      ///< Is the LED on or off? Used for flashing
enum modes { OFF, ON, FLASH };              ///< The available LED modes -- static not useful here
static enum modes mode = FLASH;             ///< Default mode is flashing

//rx buffer 
#define RX_BUFFER_SIZE 50000 
static int rx_index =0;
static unsigned int rx_buffer[RX_BUFFER_SIZE] = {2};
//static struct task_struct *task;            /// The pointer to the thread task

//fifo
static struct kfifo fifo;

//task
static struct task_struct *task;


static void inline delay_n_NOP(void)
{
	int i;
	for(i = SPI_DELAY_CNT; i>0; i--);
}


/******************* Write to ADC *********************/
static int bit_clc = 1<<BIT_CLC;
static void SPI_write_ch_addr(void)
{
	unsigned char write_byte = 0x19;
	unsigned char shift = 0x10;
    while (shift > 0) {
        writel(bit_clc, gpio2+CLEAR_OFFSET);
        delay_n_NOP();
        if ((_Bool) (write_byte & shift)) {
            writel(1<<BIT_MOSI, gpio2+SET_OFFSET);
            delay_n_NOP();
        } 
        else {
            writel(1<<BIT_MOSI, gpio2+CLEAR_OFFSET);
            delay_n_NOP();
    }
        shift >>= 1;
        writel(bit_clc, gpio2+SET_OFFSET);
    }
}

/******************* Read from ADC *********************/
static int SPI_read_from_adc(void)
{
    int value =0, index;

	writel(1<<BIT_CS, gpio1+CLEAR_OFFSET);
    delay_n_NOP();
    SPI_write_ch_addr();
    writel(bit_clc, gpio2+CLEAR_OFFSET);
    delay_n_NOP();
    writel(bit_clc, gpio2+SET_OFFSET);
    delay_n_NOP();
    for (index =0; index<11; index++)
	{
		writel(bit_clc, gpio2+CLEAR_OFFSET);
		delay_n_NOP();
		value <<= 1;
		value |= (0x1 & (readl(gpio1+READ_OFFSET)>>BIT_MISO));
		writel(bit_clc, gpio2+SET_OFFSET);
		delay_n_NOP();
	} 
	writel(bit_clc, gpio2+CLEAR_OFFSET);
	delay_n_NOP();
	writel(1<<BIT_CS, gpio1+SET_OFFSET);
	delay_n_NOP();
    return value;
}

static bool enqueue_flag=false;
static int symbol_count=0;
/***********************decide when to enqueue*******************/
/****************the preamble was defined with continuous 15 0s********/
static void pream_detection(unsigned int value)
{
    if(enqueue_flag)
    {
        kfifo_in(&fifo,&value , sizeof(value));
        if(value > 10)
        {
            symbol_count++;
            if(symbol_count >=100)
            {
                enqueue_flag=false;
                symbol_count=0;
            }
        }
        else
        {
            symbol_count=0;
        }
    }
    else
    {
        if(value<10)
        {
            symbol_count++;
            if(symbol_count >= 15)
            {
                enqueue_flag=true;
                symbol_count=0;
            }

        }
        else
        {
            symbol_count=0;
        }
    }

}

/********************* Interrupt Handler *******************/
static unsigned int temp_value;
char buf[64];
void phy_timer_handler(rtdm_timer_t *timer)
{

	//rx_buffer[rx_index] = (unsigned int)SPI_read_from_adc();
	temp_value = (unsigned int )SPI_read_from_adc();
    //sprintf(buf, "%d", temp_value);
    kfifo_in(&fifo,&temp_value , sizeof(temp_value));
    //pream_detection(temp_value); 
  //  if(rx_index >=RX_BUFFER_SIZE) { 
//		rx_index =0;
//	}
	//debugging led
    //if(mode==FLASH) ledOn = ! ledOn;
	//else if (mode == ON) ledOn = true;
	//else ledOn = false;
	//gpio_set_value(gpioLED, ledOn);

}


/********************* Thread to send a msg using socket **************/
static int rx_buffer_print(void *arg)
{
    int len;
    char buf[64];
    struct iovec iov;
    struct sockaddr_in to; 

    printk(KERN_ERR "sendthread initialized\n");
    if( sock_create( PF_INET,SOCK_DGRAM,IPPROTO_UDP,&clientsocket)<0 ){
         printk( KERN_ERR "server: Error creating clientsocket.n" );
        return -EIO;
    }
    memset(&to,0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = in_aton( "127.0.0.1" );
    /* destination address */
    to.sin_port = htons( (unsigned short) SERVERPORT );
    memset(&msg,0,sizeof(msg));
    msg.msg_name = &to;
    msg.msg_namelen = sizeof(to);

//    char * message="hello from kernel space";

    //memcpy( buf, "hello from kernel space", 24 );
    memcpy(buf, "hello from kernel space", 24);
    iov.iov_base = buf;
    iov.iov_len  = 24;
    msg.msg_control = NULL;
    msg.msg_controllen = 0;
    msg.msg_iov    = &iov;
    msg.msg_iovlen = 1;
    // msg.msg_flags    = MSG_NOSIGNAL;
    printk(KERN_ERR " vor send\n");
    oldfs = get_fs();
    set_fs( KERNEL_DS );
        len = sock_sendmsg( clientsocket, &msg, 24 );
       //sprintf(buf, "%d -- %d", i, rx_buffer[i]);
        //memcpy(buf, "hello", 24);
       // msg.msg_iov = &iov;
        msleep(1);
    set_fs( oldfs );
    printk( KERN_ERR "sock_sendmsg returned: %d\n", len);

    for(;;)
    {
        while(!kfifo_is_empty(&fifo))
        {
            unsigned int val;
            int ret;

            ret = kfifo_out(&fifo, &val, sizeof(val));
            if(ret!= sizeof(val))
            {
            //    printk(KERN_INFO "NUMBER WRONG");
                return -EINVAL;
            }
            else
            {
                 sprintf(buf, "%d",  val);
        //memcpy(buf, "hello", 24);
                 sock_sendmsg( clientsocket, &msg, 24 );
               // printk(KERN_INFO "%d\n", val);
            }
        } 
	    msleep(1);
        if(kthread_should_stop())
		    return 0;
    }
    return 0;
}


/*********************** init ******************************/
static int __init decode_init(void){
    int ret = -ENOMEM;
    printk(KERN_INFO "Initializing the decoding LKM\n");
    




    
    //gpio for led
    ledOn = false;
    gpio_request(gpioLED, "sysfs");          // gpioLED is 49 by default, request it
    gpio_direction_output(gpioLED, ledOn);   // Set the gpio to be in output mode and turn on
    gpio_export(gpioLED, false);  // causes gpio49 to appear in /sys/class/gpio
                                 // the second argument prevents the direction from being changed
   
    //gpios for SPI
    if ( gpio_request(SPI_CLC, "SPI_CLC")
        || gpio_request(SPI_MISO, "SPI_MISO")
        || gpio_request(SPI_MOSI, "SPI_MOSI")
        || gpio_request(SPI_CS, "SPI_CS") ) {
        printk("Request GPIO failed!\n");
        ret = -ENOMEM;
    }
    gpio_direction_output(SPI_CLC, false);
    gpio_direction_input(SPI_MISO);
    gpio_direction_output(SPI_MOSI, false);
    gpio_direction_output(SPI_CS, false); 

    gpio1 = ioremap(ADDR_BASE_0, 4);
    gpio2 = ioremap(ADDR_BASE_1, 4);

    //create a queue
    ret=kfifo_alloc(&fifo, PAGE_SIZE, GFP_KERNEL);
    if(ret)
    {
        printk(KERN_INFO "FIFO: error creating FIFO");
        return ret;
    }
    //timer
    slot_ns = 1000000000/SYMBOL_PERIOD;
    ret = rtdm_timer_init(&phy_timer, phy_timer_handler,"phy timer");
    if(ret)
    {
        rtdm_printk("PWM: error initializing up-timer: %i\n", ret);
        return  ret;
    }	 
    ret = rtdm_timer_start(&phy_timer, slot_ns, slot_ns, RTDM_TIMERMODE_RELATIVE);
    if(ret)
    {
        rtdm_printk("PWM: error starting up-timer: %i\n", ret);
        return ret;	
    }
    rtdm_printk("PWM: timers created\n");



   //thread
   task = kthread_run(rx_buffer_print, NULL, "print rx buffer");  // Start the LED flashing thread
   printk("Printing thread created:\n");
   if(IS_ERR(task)){                                     // Kthread name is LED_flash_thread
      printk(KERN_ALERT "EBB RX_BUFFER: failed to create the task\n");
      return PTR_ERR(task);
   }


    return 0;
}

 /***************************** exit *****************************/
static void __exit decode_exit(void){
    if(clientsocket)
        sock_release(clientsocket);

    kfifo_free(&fifo);
    kthread_stop(task);
    task = NULL;
    rtdm_timer_destroy(&phy_timer);


    iounmap(gpio1);
    iounmap(gpio2);

    gpio_set_value(gpioLED, 0);              // Turn the LED off, indicates device was unloaded
    gpio_unexport(gpioLED);                  // Unexport the Button GPIO
    gpio_free(gpioLED);                      // Free the LED GPIO
    gpio_free(SPI_CLC);
    gpio_free(SPI_MISO);
    gpio_free(SPI_MOSI);
    gpio_free(SPI_CS);
    printk(KERN_INFO "EBB LED: Goodbye from the decoding LKM!\n");
}

/********************** main *************************/
module_init(decode_init);
module_exit(decode_exit);
