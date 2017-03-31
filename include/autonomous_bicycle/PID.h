#ifndef AUTONOMOUS_BICYCLE_PID_H
#define AUTONOMOUS_BICYCLE_PID_H

#include <time.h>

class PID {
public:
    double kp, ki, kd, windupGuard;
    double setPoint, output, error;
    double sampleTime;

    PID() {}
    PID(double kp, double kd, double ki, double windupGuard) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->windupGuard = windupGuard;
        this->current_time = time(NULL);
        this->last_time = time(NULL);
    }

    void clear() {
        this->PTerm = 0;
        this->ITerm = 0;
        this->DTerm = 0;
        this->last_error = 0;
        this->output = 0;
    }

    void update(double feedback) {
        this->error = this->setPoint - feedback;
        this->current_time = time(NULL);

        double deltaTime = this->current_time - this->last_time;
        double deltaError = this->error - this->last_error;

        if (deltaTime >= this->sampleTime) {
            this->PTerm = this->kp * this->error;
            this->ITerm += this->error * deltaTime;

            if (this->ITerm < -this->windupGuard)
                this->ITerm = -this->windupGuard;

            if (this->ITerm > this->windupGuard)
                this->ITerm = this->windupGuard;

            this->DTerm = 0;
            if (deltaTime > 0) {
                this->DTerm = deltaError / deltaTime;
            }

            this->last_time = this->current_time;
            this->last_error = this->error;

            this->output = this->PTerm + (this->ki * this->ITerm) + (this->kd * this->DTerm);
        }
    }

    void setKp(double kp) {
        this->kp = kp;
    }

    void setKd(double kd) {
        this->kd = kd;
    }

    void setKi(double ki) {
        this->ki = ki;
    }

    void setWindUp(double WindUp) {
        this->windupGuard = WindUp;
    }

    void setSampleTime(double sampleTime) {
        this->sampleTime = sampleTime;
    }

private:
    float PTerm, ITerm, DTerm, last_error;
    time_t current_time, last_time;
};

#endif //AUTONOMOUS_BICYCLE_PID_H
