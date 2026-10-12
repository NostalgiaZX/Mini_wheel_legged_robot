
#include "../Inc/servo.h"
servo hip_l(1);
servo knee_l(2);
servo wheel_l(3);
servo hip_r(4);
servo knee_r(5);
servo wheel_r(6);

servo* servos[6]={
    &hip_l,
    &knee_l,
    &wheel_l,
    &hip_r,
    &knee_r,
    &wheel_r
};