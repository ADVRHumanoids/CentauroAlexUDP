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
#include <CentauroUDP/packet/slave2master.h>

#define SENDER "192.168.0.10"
#define RECEIVER "192.168.0.2"
#define BUFLEN_MASTER_2_SLAVE sizeof(CentauroUDP::packet::master2slave) 
#define BUFLEN_SLAVE_2_MASTER sizeof(CentauroUDP::packet::slave2master)
#define PORT_MASTER_2_SLAVE 16000   //The port on which to listen for incoming data
#define PORT_SLAVE_2_MASTER 16001   //The port on which to listen for incoming data

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
    struct CentauroUDP::packet::master2slave *pkt = (CentauroUDP::packet::master2slave *)malloc(BUFLEN_MASTER_2_SLAVE);
    
    // slave to master packet
    struct CentauroUDP::packet::slave2master *pkt_to_send = (CentauroUDP::packet::slave2master *)malloc(BUFLEN_SLAVE_2_MASTER);
    
    // exoskeleton pipe
    int exoskeleton_fd = open((pipe_prefix+std::string("exoskeleton_pipe")).c_str(), O_WRONLY);
    if( exoskeleton_fd < 0 ){
        die("Open exoskeleton_fd");
    }
    printf("fd : %d\n", exoskeleton_fd);
    fflush(stdout);
    
    // robot pipe
    int robot_fd = open((pipe_prefix+std::string("robot_pipe")).c_str(), O_RDONLY);
    if( robot_fd < 0 ){
        die("Open robot_fd");
    }
    printf("fd : %d\n", robot_fd);
    fflush(stdout);
     
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
     
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
    
    // initialize address to bind
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT_MASTER_2_SLAVE);
    si_me.sin_addr.s_addr = inet_addr(RECEIVER);
     
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    
    // initialize master address
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_SLAVE_2_MASTER);
    if (inet_aton(SENDER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
        
     
    //keep listening for data
    while(1)
    {
        printf("Waiting for data...");
        fflush(stdout);
         
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, pkt, BUFLEN_MASTER_2_SLAVE, 0, (struct sockaddr *) &si_other, &slen)) == -1)
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
        printf("r_position_x: %f\n" , pkt->r_position_x);
        printf("r_position_y: %f\n" , pkt->r_position_y);
        printf("r_position_z: %f\n" , pkt->r_position_z);
//         printf("r_velocity_x: %f\n" , pkt->r_velocity_x);
//         printf("r_velocity_y: %f\n" , pkt->r_velocity_y);
//         printf("r_velocity_z: %f\n" , pkt->r_velocity_z);
        
        // write on exoskeleton_pipe
//         exoskeleton_pipe.xddp_write<CentauroUDP::packet::master2slave>(*pkt);   
        int bytes = write(exoskeleton_fd, (void *)pkt, BUFLEN_MASTER_2_SLAVE);
        
        // read from robot pipe
        bytes = read(robot_fd, (void *)pkt_to_send, BUFLEN_SLAVE_2_MASTER);
        //send the message
        if (sendto(s, pkt_to_send, BUFLEN_SLAVE_2_MASTER , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
        usleep(10000);
    }
 
    close(s);
    close(robot_fd);
    close(exoskeleton_fd);
    return 0;
}
