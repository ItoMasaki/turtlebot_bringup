#include "PID.hpp"

PID::PID(float Kp, float Ki, float Kd){
    Kp = Kp;
    Ki = Ki;
    Kd = Kd;
}

float PID::average(float R, float Y){
    return Kp*(R - Y);
}


float PID::integral(float E){
    Es = Es + E;

    return Ki*Es;
}


float PID::differential(float E){
    diff = Ep - E;

    Ep = E;

    return Kd*diff;
}


float PID::system(float R, float Y){
    float P = average(R, Y);
    float I = integral(P);
    float D = differential(P);

    return P + I + D;
}
