#pragma once
#include "Tank.h"

namespace Machines {
    // Rocket Cellini Tank in mm
    // official specs: ~2900ml
    //   (189 * 69 - (3 * 3 * (4 - 3,1415))) * 250 / 1000 = 3.258,318375
    const TankGeometry RocketCelliniV2 = {
        .l = 189,
        .w = 69,
        .h = 250, // 2.9l spec = 230mm max. fillheight
        .r = 3,
        .v = 0,
        .sensorHeight = 260,
    };

    const TankGeometry LelitAnna = {
        .l = 170,
        .w = 62,
        .h = 230, // max fill height
        .r = 5,   // guess
        .v = 0,
        .sensorHeight = 250,
    };

    // Set default machine here
    inline const TankGeometry& DefaultTank = LelitAnna;
};

