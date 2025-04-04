#pragma once
#include <Arduino.h>

struct TankGeometry {
    uint16_t l;           // length in mm
    uint16_t w;           // width in mm
    uint16_t h;           // height in mm
    uint16_t r;           // corner radius in mm
    uint16_t v;           // volume in ml (calculated)
    uint16_t sensorHeight;// height of sensor mount in mm
};

class WaterTank {
public:
    // (rounded) square/rectangular tank
    WaterTank(const uint16_t& h, const uint16_t& l, const uint16_t& w, const uint16_t& r = 0);
    
    // round tank
    WaterTank(const uint16_t& h, const uint16_t& r);
    
    // construct from geometry
    WaterTank(const TankGeometry &geo);

    uint16_t fullVolume() const;
    int16_t sensorOffset() const;
    uint16_t filledVolumeTo(const uint16_t& fill_mm) const;
    uint8_t filledPercent(const uint16_t& fill_mm) const;

    const TankGeometry& geometry() const { return tank; }

    TankGeometry tank;
};
