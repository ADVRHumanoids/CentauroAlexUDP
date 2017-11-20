#include<stdio.h> //Logger::info
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>

#include <sched.h>

#include <XBotCore-interfaces/XBotPipes.h>
#include <XCM/XBotUtils.h>

#include <XBotCore-interfaces/XDomainCommunication.h>

#include <CentauroAlexUDP/packet/master2slave.h>
#include <CentauroAlexUDP/packet/slave2master.h>

#define SENDER "10.24.4.10"
#define RECEIVER "10.24.4.77"
#define BUFLEN_MASTER_2_SLAVE sizeof(CentauroUDP::packet::master2slave) 
#define BUFLEN_SLAVE_2_MASTER sizeof(CentauroUDP::packet::slave2master)
#define PORT_MASTER_2_SLAVE 16006   
#define PORT_SLAVE_2_MASTER 16106   

void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(void)
{
    // set the CPU id
    
    // UDP related stuffs
    struct sockaddr_in si_me, si_other, si_recv;
    int s, s_send, i , recv_len;
    uint slen = sizeof(si_other);
    
    // master to slave packet
    struct CentauroUDP::packet::master2slave *pkt = (CentauroUDP::packet::master2slave *)malloc(BUFLEN_MASTER_2_SLAVE);
    
    // slave to master packet
    struct CentauroUDP::packet::slave2master *pkt_to_send = (CentauroUDP::packet::slave2master *)malloc(BUFLEN_SLAVE_2_MASTER);
    
    // exoskeleton pipe   
    XBot::PublisherNRT<CentauroUDP::packet::master2slave> exoskeleton_pub("exoskeleton_pipe");
    
    // robot pipe
    XBot::SubscriberNRT<CentauroUDP::packet::slave2master> robot_sub("robot_pipe");
     
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
    
    //create send UDP socket
    if ((s_send=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
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
        Logger::error("inet_aton() failed\n");
        exit(1);
    }
        
     
    //keep listening for data
    while(1)
    {
        Logger::info("Waiting for data...");
        fflush(stdout);
         
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, pkt, BUFLEN_MASTER_2_SLAVE, 0, (struct sockaddr *) &si_recv, &slen)) == -1)
        {
            die("recvfrom()");
        }
        
        
//         Logger::info("timer master: %f - timer slave: %f \n", pkt->timer_master, (XBot::get_time_ns() / 10e9));
//         
//         // Logger::info test
        Logger::info("l_handle_trigger: %f" , pkt->l_handle_trigger);
        Logger::info("l_position_x: %f" , pkt->l_position_x);
        Logger::info("l_position_y: %f" , pkt->l_position_y);
        Logger::info("l_position_z: %f" , pkt->l_position_z);
//         Logger::info("l_velocity_x: %f\n" , pkt->l_velocity_x);
//         Logger::info("l_velocity_y: %f\n" , pkt->l_velocity_y);
//         Logger::info("l_velocity_z: %f\n" , pkt->l_velocity_z);
//         
        Logger::info("r_handle_trigger: %f" , pkt->r_handle_trigger);
        Logger::info("r_position_x: %f" , pkt->r_position_x);
        Logger::info("r_position_y: %f" , pkt->r_position_y);
        Logger::info("r_position_z: %f" , pkt->r_position_z);
//         Logger::info("r_velocity_x: %f\n" , pkt->r_velocity_x);
//         Logger::info("r_velocity_y: %f\n" , pkt->r_velocity_y);
//         Logger::info("r_velocity_z: %f\n" , pkt->r_velocity_z);
        
        // write on exoskeleton_pipe
        exoskeleton_pub.write(*pkt);
//         int bytes = write(exoskeleton_fd, (void *)pkt, BUFLEN_MASTER_2_SLAVE);
        
        // read from robot pipe
        robot_sub.read(*pkt_to_send);
//         bytes = read(robot_fd, (void *)pkt_to_send, BUFLEN_SLAVE_2_MASTER);
        
        Logger::info("force sent: %f %f %f \n" , pkt_to_send->l_force_x, pkt_to_send->l_force_y, pkt_to_send->l_force_z);
        
        
        // put back the master timer
        pkt_to_send->timer_slave = pkt->timer_master;
        //send the message
        if (sendto(s_send, pkt_to_send, BUFLEN_SLAVE_2_MASTER , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
//         usleep(1000); // 1 ms
    }
 
    close(s);
    close(s_send);

    return 0;
}
