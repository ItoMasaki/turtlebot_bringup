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

        float diff = 0;

        float system(float R, float Y);

    private:
        float average(float R, float Y);
        float integral(float E);
        float differential(float E);

};
