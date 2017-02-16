#include <linux/module.h>
#include <net/sock.h> 
#include <linux/netlink.h>
#include <linux/skbuff.h> 
#include <rtdm/rtdm_driver.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include "ofdm.h"


#define NETLINK_USER 31
#define SIZE_IFFT     64

#define GUARD_TIME    23

struct sock *nl_sk = NULL;
struct Symbol
{
	int* samples;
	int size;
};

struct node
{
    struct Symbol data;
    struct node *ptr;
}*front,*rear,*temp,*front1;
 
struct Symbol* frontelement();
void enq(struct Symbol data);
void deq();
int empty();
//void display();
void create();
int queuesize();

int count = 0;

volatile void* gpio1;
volatile void* gpio2;

static rtdm_timer_t phy_timer;
uint32_t slot_ns;

static int decoding_sleep_slot = 1;
static int frq = 2; // Unit: K

static int bit_clc = 1<<BIT_CLC;

int s_cpt = 0;
void delay_n_NOP(void)
{
    int i;
    for(i=SPI_DELAY_CNT; i>0; i--) 
        ;
}


void SPI_write(uint16_t data)
{
uint16_t shift = 0x8000; 
    while (shift > 0) {
        writel(bit_clc, gpio2+CLEAR_OFFSET);
        //clear_gpio(clc);
//      nano_sleep();
        delay_n_NOP();
        if ((data & shift) != 0) {
            writel(1<<BIT_MOSI, gpio2+SET_OFFSET);
//              printf("1\n");
            //set_gpio(mosi);
            delay_n_NOP();
        } else {
            writel(1<<BIT_MOSI, gpio2+CLEAR_OFFSET);
            //clear_gpio(mosi);
//              printf("0\n");
            delay_n_NOP();
        }
        shift >>= 1;
        writel(bit_clc, gpio2+SET_OFFSET);
        //set_gpio(clc);
//      nano_sleep();
//        delay_n_NOP();
    }

}
/* Create an empty queue */
void create()
{
    front = rear = NULL;
}
 
/* Returns queue size */
int queuesize()
{
return count;
}
 
/* Enqueing the queue */
void enq(struct Symbol data)
{
    if (rear == NULL)
    {
        rear = (struct node *)kmalloc(1*sizeof(struct node), GFP_KERNEL);
        rear->ptr = NULL;
        rear->data = data;
        front = rear;
    }
    else
    {
        temp=(struct node *)kmalloc(1*sizeof(struct node), GFP_KERNEL);
        rear->ptr = temp;
        temp->data = data;
        temp->ptr = NULL;
 
        rear = temp;
    }
    count++;
}
 
/* Displaying the queue elements */
/*void display()
{
    front1 = front;
 
    if ((front1 == NULL) && (rear == NULL))
    {
        printf("Queue is empty");
        return;
    }
    while (front1 != rear)
    {
        printf("%d ", front1->info);
        front1 = front1->ptr;
    }
    if (front1 == rear)
        printf("%d", front1->info);
}
 */
/* Dequeing the queue */
void deq()
{
    front1 = front;
 
    if (front1 == NULL)
    {
        return;
    }
    else
        if (front1->ptr != NULL)
        {
            front1 = front1->ptr;
            kfree(front);
            front = front1;
        }
        else
        {
            kfree(front);
            front = NULL;
            rear = NULL;
        }
        count--;
	//printk(KERN_INFO "node dequeued\n");
}
 
/* Returns the front element of queue */
struct Symbol* frontelement()
{
    if ((front != NULL) && (rear != NULL))
        return &(front->data);
    else
        return NULL;
}
 
/* Display if queue is empty or not */
int empty()
{
     if ((front == NULL) && (rear == NULL))
        return -1;
    else
       return 0;
}

static void hello_nl_recv_msg(struct sk_buff *skb)
{

    struct nlmsghdr *nlh;
    int pid;
    struct sk_buff *skb_out;
    int msg_size;
    char *msg = "Hello from kernel";
    int res;
    int w=0;
    //int buffer[SIZE_IFFT+GUARD_TIME];
    struct Symbol s;
    printk(KERN_INFO "Entering: %s\n", __FUNCTION__);

    msg_size = strlen(msg);

    nlh = (struct nlmsghdr *)skb->data;
    int recv_size = nlh->nlmsg_len - 4* sizeof(int);
    s.samples = (int*) kmalloc(recv_size, GFP_KERNEL);
    s.size = recv_size/sizeof(int);
    printk(KERN_INFO "sample numbers %d\n",s.size);
    memcpy(s.samples, nlmsg_data(nlh), /*(SIZE_IFFT+GUARD_TIME)*sizeof(int)*/recv_size);
    //s.samples[s.size] = 0;
    //printk(KERN_INFO "prev %d\n",);
    for(w=0;w<s.size;w++)
	printk(KERN_INFO "sample %d\n",s.samples[w]);

   printk(KERN_INFO "buffer size %d\n",nlh->nlmsg_len);
   enq(s); 
   int i=0;
    /*for(i=0;i<SIZE_IFFT+GUARD_TIME; i++)
    	printk(KERN_INFO "Netlink received msg payload:%d\n",s.samples[i]);*/
    pid = nlh->nlmsg_pid; /*pid of sending process */

    skb_out = nlmsg_new(msg_size, 0);
    if (!skb_out) {
        printk(KERN_ERR "Failed to allocate new skb\n");
        return;
    }

    nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);
    NETLINK_CB(skb_out).dst_group = 0; /* not in mcast group */
    strncpy(nlmsg_data(nlh), msg, msg_size);

    res = nlmsg_unicast(nl_sk, skb_out, pid);
    if (res < 0)
        printk(KERN_INFO "Error while sending bak to user\n");
}

void phy_timer_handler(rtdm_timer_t *timer)
{
	struct Symbol s;
	if(!empty())
	{
		s= *frontelement();
		uint16_t sample = s.samples[s_cpt];
		if(s_cpt<(s.size-1))
		{
			s_cpt++;
		}
		else
		{
			s_cpt = 0;
			deq();
		}
		//deq();
			//printk(KERN_INFO "%d\n",sample);
		writel(1<<BIT_CS, gpio1+CLEAR_OFFSET);
		delay_n_NOP();
       		sample = 0x0FFF & sample;
        	sample = 0x3000 | sample;
		SPI_write(sample);
		writel(1<<BIT_CS, gpio1+SET_OFFSET);
 		delay_n_NOP();
		//deq();
		
	}
	else
	{
		writel(1<<BIT_CS, gpio1+CLEAR_OFFSET);
                delay_n_NOP();
		uint16_t sample = 0;
                sample = 0x0FFF & sample;
                sample = 0x3000 | sample;
                SPI_write(sample);
                writel(1<<BIT_CS, gpio1+SET_OFFSET);
                delay_n_NOP();
		//printk(KERN_INFO "queue is empty, waiting for data\n");
	}
}

static int __init hello_init(void)
{
    int ret = 0;
    create();
    printk("Entering: %s\n", __FUNCTION__);
    //nl_sk = netlink_kernel_create(&init_net, NETLINK_USER, 0, hello_nl_recv_msg, NULL, THIS_MODULE);
    struct netlink_kernel_cfg cfg = {
        .input = hello_nl_recv_msg,
    };

    nl_sk = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
    if (!nl_sk) {
        printk(KERN_ALERT "Error creating socket.\n");
        return -10;
    }

    frq *= 1000; // Convert the frequency from KHz to Hz
	// May optimize this part
	/// Wait to be optimized
	decoding_sleep_slot = (1000*1/frq);
	decoding_sleep_slot = (decoding_sleep_slot>=1) ? decoding_sleep_slot : 1;
	printk("Sleep slot (while decoding) is %d ms\n", decoding_sleep_slot);

/*    /// GPIOs for the LED
if ( gpio_request(GPIO_LED_ANODE, "LED_ANODE")
|| gpio_request(GPIO_LED_CATHODE, "LED_CATHODE")
|| gpio_request(GPIO_BUFFER_CONTROL, "BUFFER_CONTROL")
|| gpio_request(GPIO_H_POWER_LED, "H_POWER_LED")
|| gpio_request(GPIO_LED_OR_PD, "LED_OR_PD")
) {
printk("Request GPIO failed!\n");
ret = -ENOMEM;
goto out;
}
gpio_direction_output(GPIO_LED_ANODE, GPIOF_INIT_LOW);
gpio_direction_output(GPIO_LED_CATHODE, GPIOF_INIT_LOW);
gpio_direction_output(GPIO_BUFFER_CONTROL, GPIOF_INIT_HIGH);
gpio_direction_output(GPIO_H_POWER_LED, GPIOF_INIT_LOW);
gpio_direction_output(GPIO_LED_OR_PD, GPIOF_INIT_LOW);
*/
/// GPIOs for SPI
if ( gpio_request(SPI_CLC, "SPI_CLC")
|| gpio_request(SPI_MISO, "SPI_MISO")
|| gpio_request(SPI_MOSI, "SPI_MOSI")
|| gpio_request(SPI_CS, "SPI_CS") ) {
printk("Request GPIO failed!\n");
ret = -ENOMEM;
goto out;
}
gpio_direction_output(SPI_CLC, GPIOF_INIT_LOW);
gpio_direction_input(SPI_MISO);
gpio_direction_output(SPI_MOSI, GPIOF_INIT_LOW);
gpio_direction_output(SPI_CS, GPIOF_INIT_LOW);

gpio1 = ioremap(ADDR_BASE_0, 4);
gpio2 = ioremap(ADDR_BASE_1, 4);

printk("my_gpio: Access address to device is:0x%x 0x%x\n",
(unsigned int) gpio1, (unsigned int) gpio2);
if (!(gpio1 && gpio2))
goto out;

// Timer
slot_ns = 1000000000 / frq;
printk("Slot in nanosecond: %d\n", slot_ns);
ret = rtdm_timer_init(&phy_timer, phy_timer_handler, "phy timer");
if(ret) {
rtdm_printk("PWM: error initializing up-timer: %i\n", ret);
return ret;
}

ret = rtdm_timer_start(&phy_timer, slot_ns, slot_ns,
RTDM_TIMERMODE_RELATIVE);
if(ret) {
rtdm_printk("PWM: error starting up-timer: %i\n", ret);
return ret;
}
rtdm_printk("PWM: timers created\n");
out:
    return 0;
}

static void __exit hello_exit(void)
{
    rtdm_timer_destroy(&phy_timer);
    printk(KERN_INFO "exiting hello module\n");
    netlink_kernel_release(nl_sk);

    iounmap(gpio1);
    iounmap(gpio2);

    gpio_free(SPI_CLC);
    gpio_free(SPI_MISO);
    gpio_free(SPI_MOSI);
    gpio_free(SPI_CS);
}

module_init(hello_init); module_exit(hello_exit);

MODULE_LICENSE("GPL");
