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


#ifndef __CENTAURO_UDP_PACKET_PEDAL_2_SLAVE_H__
#define __CENTAURO_UDP_PACKET_PEDAL_2_SLAVE_H__

namespace CentauroUDP {
    namespace packet {
        struct pedal2slave{
            int x;
            int teta;
            int y;
        };
    }   
}
#endif //__CENTAURO_UDP_PACKET_PEDAL_2_SLAVE_H__