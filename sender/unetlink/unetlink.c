#include <sys/socket.h>
#include <linux/netlink.h>
#include <stdint.h>
#include "functions.h"
#include <stdio.h>
#include <stdlib.h>
#include <fftw3.h>
#include <unistd.h>
#include <string.h> 
#include <time.h> 
#include <signal.h> 
#include <sys/time.h>

#define NETLINK_USER 31

#define MAX_PAYLOAD 1024 /* maximum payload size*/
struct sockaddr_nl src_addr, dest_addr;
struct nlmsghdr *nlh = NULL;
struct iovec iov;
int sock_fd;
struct msghdr msg;
fftw_complex *levels;


void main(int argc, char **argv)
{
    int ret = 0;
        uint8_t* data = NULL;

        long bLength;
        int nSymbols;
        fftw_complex *mod;
        fftw_complex *ifft;
        fftw_complex *Signal;
        fftw_complex *guard;

    mod = fftw_alloc_complex(SIZE_IFFT);
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

	nSymbols = (((bLength*8)%N_REAL_SUBCARRIERS) == 0 ?((bLength*8)/N_REAL_SUBCARRIERS): ((bLength*8)/N_REAL_SUBCARRIERS) +1 );

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
                performIfft(mod,ifft);
/*int k = 0;
                for(k = 0 ; k < 64 ; k++)
                {
                        printf("%d: real %lf im %lf\n",k, ifft[k][0],ifft[k][1]);
                }*/
                
                
                sum_samples(Signal, ifft, 64, (i+1)*GUARD_TIME+i*64);
                
                
        }
int j = 0;
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
                for(j = 0; j < (SIZE_IFFT+GUARD_TIME)* nSymbols; j++)
                {
                        Signal[j][0] += 64;
                        printf("%lf\n",Signal[j][0]);
                }

convert_signal_to_levels(Signal, levels, max,(SIZE_IFFT+GUARD_TIME)*nSymbols);

    sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_USER);
    if (sock_fd < 0)
        return -1;
    int fake[(SIZE_IFFT+GUARD_TIME)*nSymbols];
    int f_cpt = 0;
    for(f_cpt = 0 ; f_cpt< (SIZE_IFFT+GUARD_TIME)*nSymbols; f_cpt++)
    {
	fake[f_cpt]=(int)levels[f_cpt][0];
	//printf("val to send %d\n",fake[f_cpt]);
    }
    printf("sample numbers %d", (SIZE_IFFT+GUARD_TIME)*nSymbols);

    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = getpid(); /* self pid */

    bind(sock_fd, (struct sockaddr *)&src_addr, sizeof(src_addr));

    memset(&dest_addr, 0, sizeof(dest_addr));
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.nl_family = AF_NETLINK;
    dest_addr.nl_pid = 0; /* For Linux Kernel */
    dest_addr.nl_groups = 0; /* unicast */

//    while(1)
//    {
    nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(nSymbols*(SIZE_IFFT+GUARD_TIME)*sizeof(int)));
    memset(nlh, 0, NLMSG_SPACE(nSymbols*(SIZE_IFFT+GUARD_TIME)*sizeof(int)));
    nlh->nlmsg_len = NLMSG_SPACE(nSymbols*(SIZE_IFFT+GUARD_TIME)*sizeof(int));
    nlh->nlmsg_pid = getpid();
    nlh->nlmsg_flags = 0;
    printf("sample numbers sent %d\n", nlh->nlmsg_len/sizeof(int));
    memcpy(NLMSG_DATA(nlh), fake,nSymbols*(SIZE_IFFT+GUARD_TIME)*sizeof(int));
    //strcpy(NLMSG_DATA(nlh), fake);
	printf("size of message %d",nlh->nlmsg_len);
    iov.iov_base = (void *)nlh;
    iov.iov_len = nlh->nlmsg_len;
    msg.msg_name = (void *)&dest_addr;
    msg.msg_namelen = sizeof(dest_addr);
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;

    printf("Sending message to kernel\n");
    sendmsg(sock_fd, &msg, 0);
    printf("Waiting for message from kernel\n");

    /* Read message from kernel */
    recvmsg(sock_fd, &msg, 0);
    printf("Received message payload: %s\n", NLMSG_DATA(nlh));
    free(nlh);
//}
    close(sock_fd);
}
