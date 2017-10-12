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


#ifndef __CENTAURO_UDP_PACKET_SLAVE_2_MASTER_H__
#define __CENTAURO_UDP_PACKET_SLAVE_2_MASTER_H__

namespace CentauroUDP {
    namespace packet {
        struct slave2master{
            float timer_slave;
            float status;
            // 1 is right
            float r_force_x;
            float r_force_y;
            float r_force_z;
            // 2 is right
            float l_force_x;
            float l_force_y;
            float l_force_z;
        };
    }   
}
#endif //__CENTAURO_UDP_PACKET_SLAVE_2_MASTER_H__
