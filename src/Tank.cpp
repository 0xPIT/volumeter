#include "Tank.h"

WaterTank::WaterTank(const uint16_t& h, const uint16_t& l, const uint16_t& w, const uint16_t& r) 
    : tank { l, w, h, r, 0, h }
{
    tank.v = fullVolume();
}

WaterTank::WaterTank(const uint16_t& h, const uint16_t& r)
    : WaterTank(h, r*2, r*2, r) 
{
    tank.v = fullVolume();
}

WaterTank::WaterTank(const TankGeometry &geo) 
    : tank(geo) 
{
    tank.v = fullVolume();
}

uint16_t WaterTank::fullVolume() const {
    return filledVolumeTo(tank.h);
}

int16_t WaterTank::sensorOffset() const {
    return abs(tank.h - tank.sensorHeight);
}

uint16_t WaterTank::filledVolumeTo(const uint16_t& fill_mm) const {
    // base rectangular area
    float area = tank.l * tank.w;
    
    // subtract the four corner areas (each corner is a quarter circle)
    if (tank.r > 0) {
        area -= 4 * (tank.r * tank.r * (1.0f - PI/4.0f));
    }

    // volume in milliliters
    float volume = (area * fill_mm) / 1000.0f;

    return volume < 0 ? 0 : (uint16_t)volume;
}

uint8_t WaterTank::filledPercent(const uint16_t& fill_mm) const {
    float fill = filledVolumeTo(fill_mm);
    fill = (fill / (float)tank.v) * 100.0f;

    if (fill > 100.0f) fill = 100.0f;

    return (uint8_t)fill;
}
