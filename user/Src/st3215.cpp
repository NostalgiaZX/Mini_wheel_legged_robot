#include "../Inc/st3215.h"
//sync write
uint8_t data_write[datalen_write]={
    0xFF,0xFF,0XFE,0x16,0x83,0x2C,0x02,
    0x01,0x00,0x00,
    0x02,0x00,0x00,
    0x03,0x00,0x00,
    0x04,0x00,0x00,
    0x05,0x00,0x00,
    0x06,0x00,0x00,
    0x00
};
//sync read: addr 0x38, 15 bytes/servo, ids 1~6
uint8_t data_read[datalen_read]={
    0xFF,0xFF,0xFE,0x0A,0x82,0x38,0x0F,0x01,0x02,0x03,0x04,0x05,0x06,0x00
};

uint8_t ptrread=0;

namespace {

enum class RxState : uint8_t {
    kHeader1,
    kHeader2,
    kId,
    kLength,
    kStatus,
    kParams,
    kChecksum
};

RxState rx_state = RxState::kHeader1;
uint8_t rx_frame[READ_FRAME_LEN];
uint8_t rx_index = 0;
uint8_t rx_param_len = 0;
uint8_t rx_servo_cursor = 0; // 下一个期待到达的舵机在servos[]中的下标

// Check Sum = ~(ID + LENGTH + STATUS + Parameter1 + ... + ParameterN)
uint8_t frame_checksum(const uint8_t* frame, uint8_t total_len) {
    uint16_t sum = 0;
    for (uint8_t i = 2; i < total_len - 1; ++i) {
        sum += frame[i];
    }
    return (uint8_t)(~sum);
}

void reset_frame() {
    rx_state = RxState::kHeader1;
    rx_index = 0;
}

// 当前这一帧无法被正确识别（长度非法/校验失败/序号错位），放弃它对应的那个舵机本轮的数据，
// 但仍然要让它退出READING状态，避免整轮卡死在等待全部idle上。
void fail_current_servo() {
    if (rx_servo_cursor < SERVO_NUM) {
        servos[rx_servo_cursor]->status_.state = IDLE;
        rx_servo_cursor++;
    }
}

void handle_complete_frame() {
    const uint8_t total_len = rx_index;
    const uint8_t id = rx_frame[2];
    const uint8_t length_byte = rx_frame[3];
    const bool length_ok = (length_byte == READ_PARAM_LEN + 2);
    const bool checksum_ok = length_ok && (frame_checksum(rx_frame, total_len) == rx_frame[total_len - 1]);

    if (!length_ok || !checksum_ok) {
        fail_current_servo();
        return;
    }

    if (rx_servo_cursor >= SERVO_NUM || servos[rx_servo_cursor]->id_ != id) {
        // 乱序或多余的应答包，丢弃不处理，不更新任何状态
        return;
    }

    servo* s = servos[rx_servo_cursor];
    const uint8_t* p = &rx_frame[5];
    s->status_.error   = rx_frame[4];
    s->status_.angle   = (uint16_t)(p[0] | (p[1] << 8));
    s->status_.speed   = (uint16_t)(p[2] | (p[3] << 8));
    s->status_.load    = (uint16_t)(p[4] | (p[5] << 8));
    s->status_.voltage = p[6];
    s->status_.temp    = p[7];
    s->status_.current = p[13];
    s->status_.state = IDLE;

    rx_servo_cursor++;
}

} // namespace

void addchecksum()
{
    uint16_t sum = 0;
    for (uint16_t i = 2; i < datalen_write-1; i++)
    {
        sum += data_write[i];
    }
    data_write[datalen_write-1] = (uint8_t)(~sum);
}
void applyintensity() {

}

void servos_writing() {
    HAL_UART_Transmit(&huart1,data_write,datalen_write,100);
}
void servos_reading() {
    uint16_t sum = 0;
    for (uint16_t i = 2; i < datalen_read-1; i++)
    {
        sum += data_read[i];
    }
    data_read[datalen_read-1] = (uint8_t)(~sum);

    rx_state = RxState::kHeader1;
    rx_index = 0;
    rx_servo_cursor = 0;
    for (uint8_t i=0;i<SERVO_NUM;i++) {
        servos[i]->status_.state=READING;
    }
    HAL_UART_Transmit(&huart1,data_read,datalen_read,100);
}

uint8_t servos_all_idle() {
    //for (uint8_t i=0;i<SERVO_NUM;i++) {
    for (uint8_t i=0;i<1;i++) {
        if (servos[i]->status_.state != IDLE) {
            return 0;
        }
    }
    return 1;
}

void servos_read_timeout() {
    // 本轮超时，仍在READING的舵机视为读取失败：不更新其数据字段，仅结束本轮等待
    for (uint8_t i=0;i<SERVO_NUM;i++) {
        if (servos[i]->status_.state == READING) {
            servos[i]->status_.state = IDLE;
        }
    }
    reset_frame();
    rx_servo_cursor = SERVO_NUM;
}

void st3215_on_byte_received(uint8_t byte) {
    switch (rx_state) {
        case RxState::kHeader1:
            if (byte == 0xFF) {
                rx_index = 0;
                rx_frame[rx_index++] = byte;
                rx_state = RxState::kHeader2;
            }
            break;
        case RxState::kHeader2:
            if (byte == 0xFF) {
                rx_frame[rx_index++] = byte;
                rx_state = RxState::kId;
            } else {
                rx_state = RxState::kHeader1;
            }
            break;
        case RxState::kId:
            rx_frame[rx_index++] = byte;
            rx_state = RxState::kLength;
            break;
        case RxState::kLength:
            rx_frame[rx_index++] = byte;
            if (byte < 2 || (byte - 2) > READ_PARAM_LEN) {
                fail_current_servo();
                reset_frame();
                break;
            }
            rx_param_len = byte - 2;
            rx_state = RxState::kStatus;
            break;
        case RxState::kStatus:
            rx_frame[rx_index++] = byte;
            rx_state = (rx_param_len == 0) ? RxState::kChecksum : RxState::kParams;
            break;
        case RxState::kParams:
            rx_frame[rx_index++] = byte;
            if (rx_index >= (uint8_t)(5 + rx_param_len)) {
                rx_state = RxState::kChecksum;
            }
            break;
        case RxState::kChecksum:
            rx_frame[rx_index++] = byte;
            handle_complete_frame();
            reset_frame();
            break;
    }
}
