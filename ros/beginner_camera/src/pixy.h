//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#ifndef PIXY_H
#define PIXY_H

#include <stdio.h>
#include <stdint.h>

enum Device {NONE, PIXY, PIXY_DFU};
#define VER_MAJOR       1
#define VER_MINOR       0
#define VER_BUILD       2


#define PIXY_VID            0xb1ac
#define PIXY_DID            0xf000
#define PIXY_DFU_VID        0x1fc9
#define PIXY_DFU_DID        0x000c

#endif // PIXY_H
