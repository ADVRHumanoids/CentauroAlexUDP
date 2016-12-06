/*
    Simple udp slave
*/
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>

#include <CentauroUDP/pipes.h>

#include <CentauroUDP/packet/master2slave.h>

#define RECEIVER "192.168.0.2"
#define BUFLEN sizeof(CentauroUDP::packet::master2slave)  //Max length of buffer
#define PORT 16000   //The port on which to listen for incoming data
 
void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(void)
{
    // UDP related stuffs
    struct sockaddr_in si_me, si_other;
    int s, i , recv_len;
    uint slen = sizeof(si_other);
    
    // master to slave packet
    struct CentauroUDP::packet::master2slave *pkt = (CentauroUDP::packet::master2slave *)malloc(BUFLEN);
    
    // UDP pipe
    CentauroUDP::XDDP_pipe exoskeleton_pipe;
    exoskeleton_pipe.init( "exoskeleton_pipe");
     
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
     
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
    
    // initialize address to bind
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = inet_addr(RECEIVER);
     
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
     
    //keep listening for data
    while(1)
    {
        printf("Waiting for data...");
        fflush(stdout);
         
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, pkt, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
        {
            die("recvfrom()");
        }
         
        //print details of the client/peer and the data received
        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        
        
        printf("timer %f \n", pkt->timer_master);
        
        // printf test
//         printf("l_handle_trigger: %f\n" , pkt->l_handle_trigger);
        printf("l_position_x: %f\n" , pkt->l_position_x);
        printf("l_position_y: %f\n" , pkt->l_position_y);
        printf("l_position_z: %f\n" , pkt->l_position_z);
//         printf("l_velocity_x: %f\n" , pkt->l_velocity_x);
//         printf("l_velocity_y: %f\n" , pkt->l_velocity_y);
//         printf("l_velocity_z: %f\n" , pkt->l_velocity_z);
//         
//         printf("r_handle_trigger: %f\n" , pkt->r_handle_trigger);
//         printf("r_position_x: %f\n" , pkt->r_position_x);
//         printf("r_position_y: %f\n" , pkt->r_position_y);
//         printf("r_position_z: %f\n" , pkt->r_position_z);
//         printf("r_velocity_x: %f\n" , pkt->r_velocity_x);
//         printf("r_velocity_y: %f\n" , pkt->r_velocity_y);
//         printf("r_velocity_z: %f\n" , pkt->r_velocity_z);
        
        // write on exoskeleton_pipe
        exoskeleton_pipe.xddp_write<CentauroUDP::packet::master2slave>(*pkt);        

    }
 
    close(s);
    return 0;
}
