#ifndef __CENTAURO_UDP_PACKET_MASTER_2_SLAVE_H__
#define __CENTAURO_UDP_PACKET_MASTER_2_SLAVE_H__

namespace CentauroUDP {
    namespace packet {
        struct master2slave{
            float timer_master;
            float timer_slave;
            float run;
            // 1 is right
            float r_position_x;
            float r_position_y;
            float r_position_z;
            float r_rotation[9]; // column major ordered
            float r_velocity_x;
            float r_velocity_y;
            float r_velocity_z;
            float r_handle_trigger;
            // 2 is right
            float l_position_x;
            float l_position_y;
            float l_position_z;
            float l_rotation[9]; // column major ordered
            float l_velocity_x;
            float l_velocity_y;
            float l_velocity_z;
            float l_handle_trigger;
            
        };
    }   
}
#endif //__CENTAURO_UDP_PACKET_MASTER_2_SLAVE_H__
