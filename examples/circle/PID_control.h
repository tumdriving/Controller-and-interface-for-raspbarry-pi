#pragma

class PID
{
public:
    PID(float kp, float ki, float kd, float dt) : kp(kp), ki(ki), kd(kd), dt(dt), previous_error(0), integral(0) {}

    float calculate(float setpoint, float pv, float dt);

private:
    float kp, ki, kd, dt;
    float previous_error;
    float integral;
};

class State
{
private:
    float x = 0.0f;
    float y = 0.0f;
    float theta = 90.0f;

public:
    void getCurState(float &x, float &y, float &theta);
    void update(float vx, float vy, float omega, float dt);
};