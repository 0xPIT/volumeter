#pragma once

#include <array>
#include <algorithm>
#include <Arduino.h>

template<typename T = uint16_t>
class ExponentialAverageFilter {
    float alpha;
    T lastValue = 0;
    
public:
    ExponentialAverageFilter(float alpha_value = 0.1) : alpha(alpha_value) {}
    
    T filter(T value) {
        lastValue = alpha * value + (1 - alpha) * lastValue;
        return lastValue;
    }
};

template<size_t N, typename valueType = uint16_t>
class MedianFilter {
    std::array<valueType, N> buffer;
    size_t index = 0;
    
public:
    valueType filter(valueType value) {
        buffer[index] = value;
        index = (index + 1) % N;
        
        // Create temporary array for sorting
        std::array<valueType, N> sorted = buffer;
        std::sort(sorted.begin(), sorted.end());
        
        return sorted[N/2];
    }
};


#ifdef WITH_RATE_OF_CHANGE
template<size_t N>
class WaterLevelFilter {
    MedianFilter<N> medianFilter;
    float baseAlpha;      // base smoothing factor
    float adaptiveAlpha;  // current adaptive smoothing factor
    float lastValue = 0;
    uint32_t lastTime = 0;
    
public:
    WaterLevelFilter(float alpha_value = 0.1) : baseAlpha(alpha_value), adaptiveAlpha(alpha_value) {}
    
    uint16_t filter(uint16_t value) {
        // First remove outliers with median filter
        uint16_t medianValue = medianFilter.filter(value);
        
        // Calculate rate of change
        uint32_t currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0f; // Convert to seconds
        float rateOfChange = abs(medianValue - lastValue) / (deltaTime + 0.001f); // Avoid division by zero
        
        // Adjust alpha based on rate of change
        // Higher rate of change = higher alpha (faster response)
        // Lower rate of change = lower alpha (more smoothing)
        const float ROC_THRESHOLD = 100.0f;  // Rate of change threshold (adjust as needed)
        adaptiveAlpha = baseAlpha + (1.0f - baseAlpha) * min(1.0f, rateOfChange / ROC_THRESHOLD);
        
        // Apply EMA with adaptive alpha
        lastValue = adaptiveAlpha * medianValue + (1.0f - adaptiveAlpha) * lastValue;
        lastTime = currentTime;
        
        return static_cast<uint16_t>(lastValue);
    }
};
#else
template<size_t N, typename valueType = uint16_t>
class WaterLevelFilter {
    MedianFilter<N> medianFilter;
    float alpha;
    float lastValue = 0;
    
public:
    WaterLevelFilter(float alpha_value = 0.1) : alpha(alpha_value) {}
    
    valueType filter(valueType value) {
        // First remove outliers with median filter
        valueType medianValue = medianFilter.filter(value);
        
        // Then smooth with EMA
        lastValue = alpha * medianValue + (1 - alpha) * lastValue;
        return static_cast<valueType>(lastValue);
    }
};
#endif

class SimpleKalmanFilter {
    float q; // Process noise
    float r; // Measurement noise
    float p = 1.0; // Estimation error
    float x = 0.0; // State estimate
    float k = 0.0; // Kalman gain
    
public:
    SimpleKalmanFilter(float process_noise, float measurement_noise) 
        : q(process_noise), r(measurement_noise) {}
        
    float filter(float measurement) {
        // Prediction
        p = p + q;
        
        // Update
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;
        
        return x;
    }
}; 
