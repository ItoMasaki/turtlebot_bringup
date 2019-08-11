#include <iostream>

class PID
{
    public:

        float E, P, I, D;

        float Kp = 0.10;
        float Ki = 0.05;
        float Kd = 0.00;

        float Es = 0;
        float Ep = 0;

        float diff = 0;

        float dt = 0.02;

        float system(float R, float Y);

    private:
        float error(float R, float Y);
        float proportional(float E);
        float integral(float E);
        float differential(float E);

};
