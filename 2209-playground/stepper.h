#ifndef stepper_h
#define stepper_h
extern const bool away;
extern const bool toward;

void init_stepper();
void enable_motor();
void disable_motor();
void dance();
void move(bool dir, int distance, int delay);
void send_read_request(uint8_t address);
bool read_register(uint8_t register, uint32_t* value);
void write_register(uint8_t register, uint32_t value);
void step_motor();
void rotate(bool direction);

#endif