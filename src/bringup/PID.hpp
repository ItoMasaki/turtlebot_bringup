#include <iostream>

class PID
{
    public:
        PID(float Kp, float Ki, float Kd);
        ~PID();
        float Kp;
        float Ki;
        float Kd;

        float Es = 0;
        float Ep = 0;

    private:
        float average(float R, float Y);
        float integral(float E);
        float differential(float E);

        float system(float R, float Y);
        
};
