//
// Created by wego on 23. 6. 4.
//

#ifndef SRC_PID_H
#define SRC_PID_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd), integral(0.0), prev_error(0.0) {}

    double calculateOutput(double target, double current, double dt) {
        double error =current-target;
        std::cout<<"Error : "<<error<<std::endl;
        // 오차의 누적값 업데이트
        integral += error * dt;

        // 오차의 변화율 계산
        double derivative = (error - prev_error) / dt;
        prev_error = error;

        // 제어 출력 계산 (PID 제어 알고리즘)
        double output = Kp * error + Ki * integral + Kd * derivative;

        if(output<0.0)
            output = 0.0;

        return output;
    }

private:
    double Kp;       // P 게인
    double Ki;       // I 게인
    double Kd;       // D 게인
    double integral; // 오차의 누적값
    double prev_error; // 이전 오차 값
};

#endif //SRC_PID_H
