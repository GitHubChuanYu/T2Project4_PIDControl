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
