/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>       
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/


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
