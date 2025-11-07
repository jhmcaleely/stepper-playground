#ifndef stepper_h
#define stepper_h
extern const bool away;
extern const bool toward;

void init_stepper();
void enable_motor();
void disable_motor();
void dance();
void move(bool dir, int distance, int delay);
void send_read_request_pio(uint8_t address);
void step_motor();
void rotate(bool direction);

#endif