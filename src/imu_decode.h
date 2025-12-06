extern "C"{ 
    #include <imu.h>
};

float joycon_gyro_decode(int16_t data, enum IMUSensitivity sensitivity)
{
    switch (sensitivity)
    {
    case Sensitivity0:
        return SENSITIVITY_GYROSCOPE_245 * data;
    case Sensitivity1:
        return SENSITIVITY_GYROSCOPE_500 * data;
    case Sensitivity3:
    default:
        return SENSITIVITY_GYROSCOPE_2000 * data;
    }
}

float joycon_accel_decode(int16_t data, enum IMUSensitivity sensitivity)
{
    switch (sensitivity)
    {
    case Sensitivity0:
        return SENSITIVITY_ACCELEROMETER_8 * data;
    case Sensitivity1:
        return SENSITIVITY_ACCELEROMETER_4 * data;
    case Sensitivity2:
        return SENSITIVITY_ACCELEROMETER_2 * data;
    case Sensitivity3:
    default:
        return SENSITIVITY_ACCELEROMETER_16 * data;
    }
}