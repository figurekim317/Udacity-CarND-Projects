# Extended Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


Self-Driving Car Engineer Nanodegree Program

In this project we will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

<img src="Docs/assets/score.png" height=200px>

## Dependencies and Build Instructions
- Please check the [starter code repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) of this project for environment setup and build. 
- This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


# EKF Model Description
## State Space Vector
Basically, we first need to define the state space representation of object that we're going to track. I've defined the object as moving in **cartesian** 2-D space. The value that we want to figure out from the object is `x` and `y` **position**. Thus, the state vector consists of 
<img src="https://render.githubusercontent.com/render/math?math=x" style="background-color:white">, <img src="https://render.githubusercontent.com/render/math?math=y" style="background-color:white"> and it's first derivatives <img src="https://render.githubusercontent.com/render/math?math=\frac{dx}{dt}" style="background-color:white">, <img src="https://render.githubusercontent.com/render/math?math=\frac{dy}{dt}" style="background-color:white">

<img src="https://render.githubusercontent.com/render/math?math=x = [p_x, p_y, v_x, v_y]^T" style="margin:auto; display:block; background-color:white">

## Motion Prediction
The state transition is simply linear in 2D spaces. However, because sensors' refresh rate are **not timely constant**, we have to take these into account when dealing with state transition and variances.

The **time variant** state transition equation is defined as below.

<img src="Docs/assets/state transition eq.png" style="margin:auto; width:400px;display:block">

Also the **time variant process covariances** is derived as below. I don't write detailed derivation here. If you're interested, please check my blog [here](https://ssh199898.tistory.com/17?category=1008110) (written in Korean).

<img src="Docs/assets/process covariance.png" style="margin:auto; width:400px;display:block; ">

In this project, the acceleration covariance matrix <img src="https://render.githubusercontent.com/render/math?math=Qv" style="background-color:white"> and <img src="https://render.githubusercontent.com/render/math?math=G" style="background-color:white"> are given as below.

<img src="Docs/assets/f1.png" style="margin:auto; width:300px; display:block; background-color:white">


## Sensor Data
The data comes from two different simulated sensor model, **LIDAR** and **RADAR**.

### LIDAR Measurements
LIDAR sensor data gives the object's x and y position. Their position variances (acceleration noise) are given as `0.0225` for both <img src="https://render.githubusercontent.com/render/math?math=x, y" style="background-color:white">.

<img src="Docs/assets/f2.png" style="margin:auto; width:400px; display:block; background-color:white">


The **measurement vector** <img src="https://render.githubusercontent.com/render/math?math=z" style="background-color:white"> and **measurement matrix** <img src="https://render.githubusercontent.com/render/math?math=H" style="background-color:white"> can simply be defined as below. Remember that <img src="https://render.githubusercontent.com/render/math?math=z" style="background-color:white"> is defined as <img src="https://render.githubusercontent.com/render/math?math=z = Hx + w" style="background-color:white"> where <img src="https://render.githubusercontent.com/render/math?math=w ~ N(0, R)" style="background-color:white">

<img src="Docs/assets/f3.png" style="margin:auto; width:250px; display:block; background-color:white">


### RADAR Measurements
The need for extend kalman filters in sensor fusion comes from non-linearity of RADAR measurements. RADAR sensor gives the value in polar coordinates. It senses
 - the object's radial distance from the origin <img src="https://render.githubusercontent.com/render/math?math=\rho" style="background-color:white">
 - bearing angle <img src="https://render.githubusercontent.com/render/math?math=\phi" style="background-color:white">
 - speed in radial dirction. <img src="https://render.githubusercontent.com/render/math?math=\dot{\rho}" style="background-color:white">

Thus if we want to convert these three values, we'll have to use nonlinear functions such as *square root* and *arctan*. However, this will cause the unimodality to be broken.

To linearlize this nonlinear function, we'll use Jacobian matrix of the function at object's predicted state. The calculated Jacobian is

<img src="Docs/assets/f4.png" style="margin:auto; width:450px; display:block; background-color:white">

In the project, the variances are given as,

<img src="Docs/assets/f5.png" style="margin:auto; width:400px; display:block; background-color:white">

# Extended Kalman Filter Code
Now let's dive into C++ code that I've written. Remember EKF formula below.

<img src="Docs/assets/formula.png" style="margin:auto; width:400px;display:block">

## Motion Prediction
Prediction is performed when new measurement arrives. This function explains the whole sequence of prediction.
```C++
void KalmanFilter::Predict(float delta_T) {
    // update state transition matrix and process covariance matrix
    MatrixXd F = GetF(delta_T);
    MatrixXd Q = GetQ(delta_T);

    // predict state
    x_ = F * x_; // + v

    // predict state covariance
    P_ = F * P_ * F.transpose() + Q;
}
```
`GetF` and `GetQ` function will calculate the **time variance** state transition matrix <img src="https://render.githubusercontent.com/render/math?math=F" style="background-color:white"> and process covaraince matrix <img src="https://render.githubusercontent.com/render/math?math=Q" style="background-color:white"> based on the formulas above.
```C++
MatrixXd FusionEKF::CalculateF(float delta_T) {
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, delta_T, 0,
        0, 1, 0, delta_T,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return F;
};

MatrixXd FusionEKF::CalculateQ(float delta_T) {
    // measurement acceleration covariance
    float noise_ax = 9;
    float noise_ay = 9;

    MatrixXd Qv = MatrixXd(2, 2);
    Qv << noise_ax, 0,
        0, noise_ay;

    float sq = delta_T * delta_T / 2.0;
    MatrixXd G = MatrixXd(4, 2);
    G << sq, 0,
        0, sq,
        delta_T, 0,
        0, delta_T;

    MatrixXd Q = G * Qv * G.transpose();

    return Q;
};
```

## Measurement Update
For measurement update, we apply different pipeline by the sensor type.

### LIDAR Update
For LIDAR Update, simply perform the normal kalman filter update. It's very easy to understand the process! All matrices are constant. `GetH` and `GetR` always return same matrices.
```C++
void KalmanFilter::Update(const VectorXd &z) {
    /**
     * update the state by using Kalman Filter equations
     */
    MatrixXd H = GetH();
    MatrixXd R = GetR();
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    // measurement update
    VectorXd y = z - H * x_;
    MatrixXd S = H * P_ * H.transpose() + R;
    MatrixXd K = P_ * H.transpose() * S.inverse();

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I - K * H) * P_;
}
```
### RADAR Update
This is where the nonlinearity comes in. Because the measurement matrix <img src="https://render.githubusercontent.com/render/math?math=H_j" style="background-color:white"> is now a function of predicted state <img src="https://render.githubusercontent.com/render/math?math=x" style="background-color:white">, we'll have to re-calculate it using <img src="https://render.githubusercontent.com/render/math?math=x" style="background-color:white"> whenever RADAR measurements comes in. New verison of update function now have new functions for it.
```C++
void KalmanFilter::UpdateEKF(const VectorXd &z, 
                            VectorXd (*GetY)(const VectorXd&, const VectorXd&), 
                            MatrixXd (*GetHj)(const VectorXd&),
                            MatrixXd (*GetR)()) {
    /**
     * update the state by using Extended Kalman Filter equations
     */
    MatrixXd Hj = GetHj(x_);           // calculate jacobian using current state
    MatrixXd R = GetR();               // overrides member function
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    // measurement update
    VectorXd y = GetY(z, x_);          //y = z-hx, with angle normailzation
    MatrixXd S = Hj * P_ * Hj.transpose() + R;      //use Hj
    MatrixXd K = P_ * Hj.transpose() * S.inverse(); //use Hj

    // new estimate
    x_ = x_ + (K * y);
    P_ = (I - K * Hj) * P_;                         //use Hj
}
```
Let's take a glimpse into what `GetHj()` does. Okay... it seems to be just a hardcoded jacobian matrix! 
```C++
MatrixXd FusionEKF::CalculateRadarJacobian(const VectorXd &x_state) {
    MatrixXd Hj = MatrixXd(3, 4);
    Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // if divided by zero, the measurement is overlaped with the car
    if (px * px + py * py == 0)
    {
        return Hj; //return zero value
    }

    float p_abs = sqrt(px * px + py * py);

    // compute the Jacobian matrix
    // derivative of rho
    Hj(0, 0) = px / p_abs;
    Hj(0, 1) = py / p_abs;

    // derivative of pi
    Hj(1, 0) = -py / (px * px + py * py);
    Hj(1, 1) = px / (px * px + py * py);

    // derivative of rho dot
    Hj(2, 0) = py * (vx * py - vy * px) / pow(p_abs, 3);
    Hj(2, 1) = px * (vy * px - vx * py) / pow(p_abs, 3);
    Hj(2, 2) = px / p_abs;
    Hj(2, 3) = py / p_abs;

    return Hj;
}
```

# Demo
We're all prepared. Here we go! Click the video thumbnail to watch demo in youtube.

> I was able to reach **RMSE** of `0.0973`!!



<img src="Docs/assets/score.png" height=200px> <a href="https://youtu.be/o_1B-qBQATg"><img src="Docs/assets/youtube.png" height="200px">
