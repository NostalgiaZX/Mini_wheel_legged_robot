
#ifndef MINI_WHEEL_LEGGED_ROBOT_SERVO_H
#define MINI_WHEEL_LEGGED_ROBOT_SERVO_H
#include <cstdint>

enum work_status {
    IDLE,
    WRITING,
    READING
};

struct servo_status {
    work_status state;
    uint16_t angle;
    uint16_t speed;
    uint16_t load;
    uint8_t voltage;
    uint8_t temp;
    uint8_t error;
    uint8_t current;
}typedef ss;
class servo {
public:
    servo(uint8_t id) {
        id_=id;
    };
    uint8_t id_;
    ss status_;
    float intensity_;

};
extern servo* servos[6];

#endif //MINI_WHEEL_LEGGED_ROBOT_SERVO_H