#include "PID.hpp"

float PID::error(float R, float Y){
    return R - Y;    
}

float PID::proportional(float E){
    return Kp*E;
}


float PID::integral(float E){
    Es += E*dt;

    return Ki*Es;
}


float PID::differential(float E){
    diff = (Ep - E)/dt;

    Ep = E;

    return Kd*diff;
}


float PID::system(float R, float Y){
        E = error(R, Y);
        P = proportional(E);
        I = integral(E);
        D = differential(E);

    return P + I + D;
}
