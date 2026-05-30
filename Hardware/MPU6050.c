#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "MyI2C.h"
#include "delay.h"

#define MPU6050_ADDRESS        0xD0
#define MPU6050_ID             0x68
#define GYRO_SENSITIVITY       65.5f
#define DT                     0.02f

static int16_t gyro_z_bias = 0;
static float Yaw = 0.0f;
uint8_t imu_init_ok = 0;

static float kalman_x = 0.0f;
static float kalman_P = 0.0f;
static float kalman_Q = 0.02f;
static float kalman_R = 0.5f;
static uint8_t kalman_first = 1;

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return Data;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    uint8_t i;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    for (i = 0; i < Count; i++)
    {
        DataArray[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(i < Count - 1 ? 0 : 1);
    }
    MyI2C_Stop();
}

uint8_t MPU6050_Init(void)
{
    uint16_t timeout = 0;

    MyI2C_Init();
    Delay_ms(20);

    while (MPU6050_GetID() != MPU6050_ID)
    {
        Delay_ms(1);
        if (++timeout > 255)
        {
            imu_init_ok = 0;
            return 1;
        }
    }

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);

    imu_init_ok = 1;
    return 0;
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];

    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

    *AccX = (int16_t)((uint16_t)Data[0] << 8 | Data[1]);
    *AccY = (int16_t)((uint16_t)Data[2] << 8 | Data[3]);
    *AccZ = (int16_t)((uint16_t)Data[4] << 8 | Data[5]);

    *GyroX = (int16_t)((uint16_t)Data[8] << 8 | Data[9]);
    *GyroY = (int16_t)((uint16_t)Data[10] << 8 | Data[11]);
    *GyroZ = (int16_t)((uint16_t)Data[12] << 8 | Data[13]);
}

void MPU6050_Kalman_Init(float Q, float R)
{
    kalman_Q = Q;
    kalman_R = R;
    kalman_x = 0.0f;
    kalman_P = 1.0f;
    kalman_first = 1;
}

static void Kalman_SetAdaptiveQ(float raw_dps)
{
    float abs_dps = raw_dps;
    if (abs_dps < 0.0f) abs_dps = -abs_dps;

    if (abs_dps < 5.0f)
        kalman_Q = 0.02f;
    else if (abs_dps < 30.0f)
        kalman_Q = 0.1f;
    else
        kalman_Q = 0.5f;
}

static float Kalman_Update(float measurement)
{
    float K;

    if (kalman_first)
    {
        kalman_x = measurement;
        kalman_first = 0;
        return kalman_x;
    }

    kalman_P += kalman_Q;
    K = kalman_P / (kalman_P + kalman_R);
    kalman_x += K * (measurement - kalman_x);
    kalman_P *= (1.0f - K);

    return kalman_x;
}

void MPU6050_CalibrateGyroZ(void)
{
    int i;
    int32_t sum_gz = 0;
    int16_t GX, GY, GZ;
    int16_t AX, AY, AZ;

    for (i = 0; i < 100; i++)
    {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        sum_gz += GZ;
        Delay_ms(2);
    }
    gyro_z_bias = (int16_t)(sum_gz / 100);

    MPU6050_Kalman_Init(0.02f, 0.5f);
}

#define ZUPT_THRESHOLD_DPS     0.6f
#define ZUPT_COUNT_THRESHOLD   5
static uint8_t zupt_count = 0;
static float bias_correction = 0.0f;
static uint8_t was_stationary = 0;

#define TURN_HIGH_SPEED_THRESHOLD  30.0f
#define ZUPT_COOLDOWN_CYCLES       25
static uint8_t turn_cooldown = 0;

void MPU6050_UpdateYaw_Filtered(int16_t GZ)
{
    float raw_dps;
    float filtered_dps;
    float abs_raw_dps;

    raw_dps = (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY;

    Kalman_SetAdaptiveQ(raw_dps);
    filtered_dps = Kalman_Update(raw_dps);

    abs_raw_dps = raw_dps;
    if (abs_raw_dps < 0.0f) abs_raw_dps = -abs_raw_dps;

    if (abs_raw_dps >= TURN_HIGH_SPEED_THRESHOLD)
    {
        turn_cooldown = ZUPT_COOLDOWN_CYCLES;
    }
    else if (turn_cooldown > 0)
    {
        turn_cooldown--;
    }

    if (turn_cooldown == 0)
    {
        if (filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS)
        {
            if (zupt_count < ZUPT_COUNT_THRESHOLD)
                zupt_count++;

            if (zupt_count >= ZUPT_COUNT_THRESHOLD)
            {
                was_stationary = 1;

                bias_correction += filtered_dps * 0.1f;
                if (bias_correction > 0.25f)
                {
                    gyro_z_bias += 1;
                    bias_correction = 0.0f;
                }
                else if (bias_correction < -0.25f)
                {
                    gyro_z_bias -= 1;
                    bias_correction = 0.0f;
                }

                return;
            }
        }
        else
        {
            zupt_count = 0;

            if (was_stationary)
            {
                was_stationary = 0;
                bias_correction = 0.0f;
            }
        }
    }
    else
    {
        if (!(filtered_dps > -ZUPT_THRESHOLD_DPS && filtered_dps < ZUPT_THRESHOLD_DPS))
        {
            zupt_count = 0;
        }

        if (was_stationary)
        {
            was_stationary = 0;
            bias_correction = 0.0f;
        }
    }

    Yaw += filtered_dps * DT;

    if (Yaw >= 360.0f) Yaw -= 360.0f;
    if (Yaw < 0.0f) Yaw += 360.0f;
}

void MPU6050_UpdateYaw(int16_t GZ)
{
    Yaw += (float)(GZ - gyro_z_bias) / GYRO_SENSITIVITY * DT;

    if (Yaw >= 360.0f) Yaw -= 360.0f;
    if (Yaw < 0.0f) Yaw += 360.0f;
}

float MPU6050_GetYaw(void)
{
    return Yaw;
}

int16_t MPU6050_GetGyroZBias(void)
{
    return gyro_z_bias;
}
