#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>

#define PAGE_SIZE 4096

int main ( int argc, char **argv )
{
    int configfd;
    int i;
    i = 0;
    configfd = open("/sys/kernel/debug/mmap_example", O_RDWR);
    if(configfd < 0) {
        perror("open");
        return -1;
    }

    char * address = NULL;
    address = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 0);
    if (address == MAP_FAILED) {
        perror("mmap");
        return -1;
    }
    while(i<50000)
    {
        printf("%d initial message: %s\n", i, address);
        i++;
        //memcpy(address + 11 , "*user*", 6);
        //printf("changed message: %s\n", address);
    }
    close(configfd);    
    return 0;
}

