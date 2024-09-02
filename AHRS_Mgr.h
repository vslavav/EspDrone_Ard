void AHRS_Mgr_update_top(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void AHRS_Mgr_update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void AHRS_Mgr_estimatedGravityDirection(float* gx, float* gy, float* gz);
void AHRS_Mgr_GetEulerRPY(float* roll, float* pitch, float* yaw);