#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void Mahony_GetQuaternion(float *q);
void Mahony_GetEulerAngle(float *pitch, float *roll, float *yaw);

#endif