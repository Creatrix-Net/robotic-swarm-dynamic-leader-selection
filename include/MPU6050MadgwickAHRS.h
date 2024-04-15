#ifndef MPU6050MadgwickAHRS_h
#define MPU6050MadgwickAHRS_h

class MPU6050MadgwickAHRS {
  public:
    MPU6050MadgwickAHRS();
    void update();
    float getRollU();
    float getPitchU();
    float getYawU();
    float getRollRadiansU();
    float getPitchRadiansU();
    float getYawRadiansU();
    float convertRawAcceleration(int aRaw);
    float convertRawGyro(int gRaw);
  private:
    float ax, ay, az;
    float gx, gy, gz;
};

#endif