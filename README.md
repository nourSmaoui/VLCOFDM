# Software based OFDM implementation for Visible Light Communication

## Description

In this project, we aim to create a software implementation of the physical layer protocol OFDM dedicated for
visible light communication over a low ressource platform (Beagle Bone Black). The implementation includes two parts:
 
 * The receiver: Divided into a kernel module sampling the light signal and a user space program that displays the received samples.
 * The sender: Divided into a user space program that generates the OFDM signal samples and a kernel module that sends the samples through lighting intensity levels.




### Quick start

 * cd receiver
 * make
 * sudo insmod fifo_skt.ko
 * sudo ./udp_server 5555

 * cd sender
 * cd knetlink
 * make
 * sudo insmod netlink.ko
 * cd ../unetlink
 * make
 * ./unetlink testfile
 
