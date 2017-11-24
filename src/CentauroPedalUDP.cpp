#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>

#include <sched.h>

#include <XBotCore-interfaces/XBotPipes.h>
#include <XCM/XBotUtils.h>

#include <CentauroAlexUDP/packet/pedal2slave.h>
#include <CentauroAlexUDP/packet/slave2pedal.h>

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#define SENDER "10.24.4.40"
#define RECEIVER "10.24.4.77"
#define BUFLEN_PEDAL_2_SLAVE sizeof(CentauroUDP::packet::pedal2slave) 
#define BUFLEN_SLAVE_2_PEDAL sizeof(CentauroUDP::packet::slave2pedal)
#define PORT_PEDAL_2_SLAVE 15006   
#define PORT_SLAVE_2_PEDAL 15106   

#define FULL_SCALE 100

void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "centauro_pedal_udp");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/centauro_wheeled_locomotion/vref", 1, false);
    
    const double v_max = 0.3;
    const double omega_max = 1.0;
    
    // UDP related stuffs
    struct sockaddr_in si_me, si_other, si_recv;
    int s, s_send, i , recv_len;
    uint slen = sizeof(si_other);
    
    // master to slave packet
    struct CentauroUDP::packet::pedal2slave *pkt = (CentauroUDP::packet::pedal2slave *)malloc(BUFLEN_PEDAL_2_SLAVE);
    
    // slave to master packet
    struct CentauroUDP::packet::slave2pedal *pkt_to_send = (CentauroUDP::packet::slave2pedal *)malloc(BUFLEN_SLAVE_2_PEDAL);
     
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
    si_me.sin_port = htons(PORT_PEDAL_2_SLAVE);
    si_me.sin_addr.s_addr = inet_addr(RECEIVER);
     
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    
    // initialize master address
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_SLAVE_2_PEDAL);
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
        if ((recv_len = recvfrom(s, pkt, BUFLEN_PEDAL_2_SLAVE, 0, (struct sockaddr *) &si_recv, &slen)) == -1)
        {
            die("recvfrom()");
        }
        
        
        printf("\nx: %d \ny: %d \nteta: %d\n", pkt->x, pkt->y, pkt->teta);
        geometry_msgs::Twist msg;
        msg.linear.x = pkt->x * v_max / FULL_SCALE;
        msg.linear.y = pkt->y * v_max / FULL_SCALE;
        msg.angular.z = pkt->teta * omega_max / FULL_SCALE;
        
        
        // HACK 
        
        
        pub.publish(msg);
        
        //send the message
        if (sendto(s_send, pkt_to_send, BUFLEN_SLAVE_2_PEDAL , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
//         usleep(1000); // 1 ms
    }
 
    close(s);
    close(s_send);
    return 0;
}
