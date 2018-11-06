# CarND-Controls-PID
This is Chuan's writeup report for Udacity self-driving car nano degree program term 2 project 4 PID Controller

---
[//]: # (Image References)

[image1]: ./PID.png "PID"

## PID Control Implementation

In this project, PID control is used to control a vehicle's steering based on the cross track error (CTE) to make sure the vehicle follows the track as much as possible and stays inside the lane. The control input can be calculated as:

![alt text][image1]

where control input steering wheel angle equals proportional gain times CTE plus integral gain times integration of previous CTE, and plus derivative gain times derivative of CTE.

### C++ code for PID control

The code is inside PID.cpp file as shown in these two parts:

* Error updates:
```sh
void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}
```
where p_error is the error for proportional gain, i_error is the error for integral gain which means the sum of all previous CTE, and d_error is the derivative of CTE which means the difference between current and previous CTE.

* Control updates:
```sh
double PID::TotalError() {
    double steer_value;
    steer_value = - Kp * p_error - Ki * i_error - Kd * d_error;
    return steer_value;
}
```
where Kp, Ki, and Kd are relatively proportional, integral, and derivative gains.

## PID control gain tuning

In this project, the PID control gain tuning is done manually based on the principle and understanding of each gain effect on final performance. 

* Proportional gain Kp:

As shown in Udacity class, proportional gain Kp is the basic control to make sure output coverges to reference line. However if Kp is very large, it can cause a lot of overshoot and oscillation. For example, when I first tried Kp = 1.0, the short video [video 1](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_1_0.mp4) shows the vehicle tries to move towards center of the lane but with a lot of oscillations and eventually cannot go back to the lane. So I reduce the Kp a little bit to 0.25 to reduce the overshoot and in [video 2](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_0_2_5.mp4), then we can see the overshoot reduces a little bit but the vehicle still cannot stay on the lane.

* Derivative gain Kd:

Derivative gain Kd can be very useful to reduce the overshoot since it is used to multiply the derivative of error and then reduce the control input when error is reducing. So first Kp = 0.75 is tried with Kp further reduced to 0.15. The combined PD control is tested with result shown in [Video 3](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_0_1_5_Kd_0_7_5.mp4). As you can see now, the vehicle is much stable staying inside lane. Thus, Kp is further reduced to 0.1 and Kd is further increased to 1.5 to see the effect, the result in [Video 4](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_0_1_Kd_1_5.mp4) shows better performance that the vehicle can still stay inside lane in curve track.

* Integral gain Ki:

Integral gain Ki is used to deal with systematic bias which means even with PD control, the vehicle can drive inside lane but still has a constant systematic bias as to the center of the lane. Here Ki is tried with 0.01, 0.001, and 0.0005 respectively with Kp further reduced to 0.09 and Kd further increased to 1.7. The results for these three combinations of PID gains are shown:

Kp = 0.09; Ki = 0.01; Kd = 1.7: [Video 5](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_009_Ki_001_Kd_170.mp4)

Kp = 0.09; Ki = 0.001; Kd = 1.7: [Video 6](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_009_Ki_0001_Kd_170.mp4)

Kp = 0.09; Ki = 0.0005; Kd = 1.7: [Video 7](https://github.com/GitHubChuanYu/T2Project4_PIDControl/blob/master/PID_Kp_009_Ki_00005_Kd_170.mp4)

It turns out Ki = 0.0005 gives us the best result with vehicle stays in the center of lane most of time.

So the best tuned PID control gains are **Kp = 0.09; Ki = 0.0005; Kd = 1.7**.

## Future work and potential improvements
In the project, the fine tuning of PID gains can be done using the twiddle logic introduced in class, however due to time limit and limited knowledge of C++ coding experience, it is hard to write C++ codes to iterate between simulator and twiddle logic and debug it to make it work to do fine tuing of PID gains. This can definitely be a future work to set up automatical tuing using twiddle logic.
