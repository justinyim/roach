//vr_telem.h , VelociRoACH specific telemetry packet format header

#include <stdint.h>

// Data structure type

typedef struct {
    int32_t posTail;
    int32_t posFemur;
    int32_t posMotor; // Hall angle position of BLDC

    int32_t pitch; // estimated angles
    int32_t roll;
    int32_t yaw;
    int32_t pitchSet; // Commanded Hall angle position 
 
    int16_t dcBLDC;  // Current draw of BLDC

    int16_t dcTail; // PWM duty cycle
    int16_t dcProp1; // PWM duty cycle
    int16_t dcProp2; // PWM duty cycle

    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    // 2017-2018 modes. Above modes should be maintained for backwards compatibility.

    // Base new modes
    uint16_t otherMode; // Other modes (telemetry and commands)
    uint16_t onboardMode; // Onboard modes, discrete states, etc.
    uint16_t voltage; // Battery voltage 

    // Additional modes
    uint16_t crank; // Crank angle
    int16_t force; // Foot force
    int16_t foot; // Foot distance
    int16_t footVel; // Mechanical advantage
} vrTelemStruct_t;

//void vrTelemGetData(unsigned char* ptr);
void vrTelemGetData(vrTelemStruct_t* ptr);

unsigned int vrTelemGetSize();
