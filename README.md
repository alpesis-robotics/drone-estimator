# Drone Estimator

## Getting Started

```
    # install Qt5
    $ export Qt5Core_DIR=/usr/local/opt/qt/lib/cmake/Qt5Core
    $ export Qt5Network_DIR=/usr/local/opt/qt/lib/cmake/Qt5Network
    $ export Qt5Widgets_DIR=/usr/local/opt/qt/lib/cmake/Qt5Widgets

    $ mkdir _build && cd _build
    $ cmake ..
    $ make
    $ ./CPPSim
```

Simulator Commands

- ``right click``: choose scenario;
- ``left drag``: rotate;
- ``X + left drag``: pan;
- ``arrow keys``: apply external force;
- ``C``: clear all graphs;
- ``R``: reset simulation;
- ``Space``: pause simulation.

## Codes

```
    drone-controller/
          +---- images/              scenario images
          +---- config/              configuration files for controller and vehicle
          +---- lib/                 external libraries
          +---- project/             IDE configurations
          +---- src/                 codes
          +---- tools/               tools
          +---- CMakeLists.txt
          +---- README.rst
```

## Kalam Filter Process

![kalman_filter_process](./images/kalman_filter_process.png)

## Solution: Scenario 06_SensorNoise

By calculating the standard deviation of ``Quad.GPS.X`` and ``Quad.IMU.AX`` data located at ``config/log/``,
the charts in the scenario ``06_SensorNoise`` would be shown that the dashed lines turn green within +/- 1
sigma bound on the y-axis capturing the value of approximately 68% of the respective measurements.

The formula of standard deviation is as following:


![equation](http://latex.codecogs.com/gif.latex?\mu=\frac{1}{N}{\sum_{i=1}^N}x_i)

![equation](http://latex.codecogs.com/gif.latex?\sigma=\sqrt{\frac{1}{N}\sum_{i=1}^N(x_i-\mu)^2})  

With the Numpy API ``numpy.std(data)``, the implemention is located at ``tools/06_SensorNoise``:

```
import numpy as np

class SensorNoise(object):

    def __init__(self, datafile):
        self.data = np.loadtxt(datafile, delimiter=',', skiprows=1)

    def mean(self):
        return np.mean(self.data[:, 1])

    def std(self):
        return np.std(self.data[:, 1])


if __name__=='__main__':
    gpsfile = "../config/log/Graph1.txt"
    gps = SensorNoise(gpsfile)
    print "Quad.GPS.X std: ", gps.std()

    imufile = "../config/log/Graph2.txt"
    imu = SensorNoise(imufile)
    print "Quad.IMU.AX std: ", imu.std()
```

Run the result:

```
# the results will be updated instantly, corresponding to the data grows.
$ python 06_SensorNoise.py
Quad.GPS.X std:  0.71384784768
Quad.IMU.AX std:  0.511644801594

$ ./CPPSim
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 70% of the time
```

The figures turn green as shown below:

![06_SensorNoise](./images/06_SensorNoise.png)


## Solution: Scenario 07_AttitudeEstimation

With the reference of the paper [Attitude Estimation Control of Autonomous Aerial Vehicles](https://tel.archives-ouvertes.fr/tel-01201539/document), 
the attitude kinematics using the minimal Euler angles parametrization is defined as following:

![equation](http://latex.codecogs.com/gif.latex?\dot\phi(t)=\omega_x+\sin(\phi)\tan(\theta)\omega_y+\cos(\phi)\tan(\theta)\omega_z)

![equation](http://latex.codecogs.com/gif.latex?\dot\theta(t)=\cos(\phi)\omega_y-\sin(\phi)\omega_z)

![equation](http://latex.codecogs.com/gif.latex?\dot\psi(t)=\frac{\sin(\phi)}{\cos(\theta)}\omega_y+\frac{\cos(\phi)}{\cos(\theta)}\omega_z)

Notation reference:

- x axis: roll, phi
- y axis: pitch, theta
- z axis: yaw, psi

Codes implemented in ``UpdateFromIMU()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   // SMALL ANGLE GYRO INTEGRATION:
   // (replace the code below)
   // make sure you comment it out when you add your own code -- otherwise e.g. you might integrate yaw twice

   // ref: 1.4 Attitude kinematics and dynamics, Attitude Estimation Control of Autonomous Aerial Vehicles, P25
   // url: https://tel.archives-ouvertes.fr/tel-01201539/document
   V3F angleDot;
   angleDot.x = gyro.x + sin(rollEst) * tan(pitchEst) * gyro.y + cos(rollEst) * tan(pitchEst) * gyro.z;
   angleDot.y = cos(rollEst) * gyro.y - sin(rollEst) * gyro.z;
   angleDot.z = sin(rollEst) * gyro.y / cos(pitchEst) + cos(rollEst) * gyro.z / cos(pitchEst);

   float predictedPitch = pitchEst + dtIMU * angleDot.y;
   float predictedRoll = rollEst + dtIMU * angleDot.x;
   ekfState(6) = ekfState(6) + dtIMU * angleDot.z;       // yaw
   // float predictedPitch = pitchEst + dtIMU * gyro.y;
   // float predictedRoll = rollEst + dtIMU * gyro.x;
   // ekfState(6) = ekfState(6) + dtIMU * gyro.z;        // yaw

   // normalize yaw to -pi .. pi
   if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
   if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Run the result:

```
$ make
$ ./CPPSim
Simulation #1 (../config/07_AttitudeEstimation.txt)
Simulation #2 (../config/07_AttitudeEstimation.txt)
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
Simulation #3 (../config/07_AttitudeEstimation.txt)
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
...
```

A green box is shown at the chart of estimated attitude error:

![07_AttitudeEstimation](./images/07_AttitudeEstimation.png)

## Solution: Scenarios 08_PredictState & 09_PredictCovariance

In general, there are two parts in the prediction phase of the Extended Kalman Filter: state estimated and
covariance that measures the errors. The presentation could be written like:

![equation](http://latex.codecogs.com/gif.latex?X_t=F(X_{t-1},U_t)+W_t)

![equation](http://latex.codecogs.com/gif.latex?P_t=G(P_{t-1})+Q_t)

Where

- X_t: the state at time t;
- F(X, U): the state transition function;
- W_t: the state noise at time t;
- P_t: the covariance at time t;
- G(X): the Jocobian function;
- Q_t: the covariance noise at time t.

At the stage of this project, there are three steps to finish the prediction phase in the ``QuadEstimatorEKF.cpp``:

- ``Predict()``: the main prediction function, returning ``ekfState`` and ``ekfCov``;
- ``PredictState()``: the calculation of ``ekfState``;
- ``GetRbgPrime()``: the calculation of the partial derivative of the Rbg matrix for ``ekfCov``.

## ekfState

Regarding to the ``ekfState`` in the ``PredictState()``, the formulas applied are separated by position(x, y, z)
and velocity(x, y, z) as following:

![equation](http://latex.codecogs.com/gif.latex?P[x,y,z]_t=P[x,y,z]_{t-1}+V[x,y,z]_{t-1}%5CDelta%20t)

![equation](http://latex.codecogs.com/gif.latex?V[x,y]_t=V[x,y]_{t-1}+A[x,y]_{t-1}%5CDelta%20t)

![equation](http://latex.codecogs.com/gif.latex?V[z]_t=V[z]_{t-1}+A[z]_{t-1}%5CDelta%20t-g%5CDelta%20t)

Where

- P[x, y, z]: the position on x/y/z axis;
- V[x, y, z]: the velocity on x/y/z axis;
- A[x, y, z]: the acceleration on x/y/z axis;
- g: the gravity.

## ekfCov

Concerning on the ``ekfCov`` in the ``Predict()`` and ``GetRbgPrime``, the formulas implemented correspondingly as:

**Step 1. RbgPrime**

Calculating the partial derivative of the rotation matrix from body frame to global frame by the roll, pitch and yaw values.

![RbgPrime](./images/rbg_prime.gif)

**Step 2. gPrime**

Getting the Jacobian matrix by the partial derivative of the rotation matrix, acceleration and delta t.

![jacobian](./images/jacobian.gif)

**Step 3. ekfCov**

Finishing the covariance of prediction.

![equation](http://latex.codecogs.com/gif.latex?P_t=GP_{t-1}G^T+Q)

Where ``G`` is the Jacobian function calculated at step 2.

### Implementation

Come back to the project, codes implemented in ``PredictState()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   V3F curAccel = attitude.Rotate_BtoI(accel);

   predictedState(0) = curState(0) + curState(3) * dt;
   predictedState(1) = curState(1) + curState(4) * dt;
   predictedState(2) = curState(2) + curState(5) * dt;
   predictedState(3) = curState(3) + curAccel.x * dt;
   predictedState(4) = curState(4) + curAccel.y * dt;
   predictedState(5) = curState(5) + curAccel.z * dt - CONST_GRAVITY * dt;
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```


Running the result, the true (y, vy, z, vz) and the estimated (y, vy, z, vz) would 
be apporximately overlapped. 

![08_PredictState](./images/08_PredictState.png)

Codes implemented in ``GetRbgPrime()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   float sinPhi = sin(roll);
   float cosPhi = cos(roll);
   float sinTheta = sin(pitch);
   float cosTheta = cos(pitch);
   float sinPsi = sin(yaw);
   float cosPsi = cos(yaw);

   RbgPrime(0, 0) = - cosTheta * sinPsi;
   RbgPrime(0, 1) = - sinPhi * sinTheta * sinPsi - cosPhi * cosPsi;
   RbgPrime(0, 2) = - cosPhi * sinTheta * sinPsi + sinPhi * cosPsi;
   RbgPrime(1, 0) = cosTheta * cosPsi;
   RbgPrime(1, 1) = sinPhi * sinTheta * cosPsi - cosPhi * cosPsi;
   RbgPrime(1, 2) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Codes implemented in ``Predict()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   gPrime(0, 3) = dt;
   gPrime(1, 4) = dt;
   gPrime(2, 5) = dt;
   gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;
   gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;
   gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;

   ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Fine tunning the parameters in ``config/QuadEstimatorEKF.txt``:

```
QPosXYStd = .02
QVelXYStd = .18
```

![09_PredictCovariance](./images/09_PredictCovariance.png)

## Solution: Scenario 10_MagUpdate

The generic process of the update phase in Extended Kalman Filter is divided into two steps:

- Stated updated: to update the state based on predicted state, Kalman gain, the measurement
noise and the measurement prediction based on current state;
- Covariance updated: to get the covariance by the Kalman gain, covariance function and the
predicted current covariance.

The formulas represneted as listed:

![equation](http://latex.codecogs.com/gif.latex?X_t=X^{'}+K(Z_t-CX^{'}))

![equation](http://latex.codecogs.com/gif.latex?P_t=(I-KH)P^{'})

![equation](http://latex.codecogs.com/gif.latex?K_t=\frac{P^{'}H}{HP^{'}H^T+R})

Where

- X_t: the updated state;
- P_t: the updated covariance;
- X': the predicted state;
- P': the predicted covariance;
- K: the Kalman gain;
- Z_t: the measurement;
- H: the Jacobian of observation function;
- R: the measurement covariance;
- C: the constant. 

Regarding the project, the main update process is implemented in ``Predict()``, while the input
procedure of Magnetometer is the outputs of ``UpdateFromMag()`` that aims to solve the ``Z_t``,
``H``, ``R`` and ``CX'``, in the function, ``z``, ``hPrime``, ``R_Mag`` and ``zFromX`` corespondingly.

Here, the values are as below:

- ``z``: the magYaw;
- ``R_Mag``: the magnetomer measurement covariance;
- ``zFromX``: the normalized predicted yaw in ekfState (ekfState(6));
- ``hPrime``: 

![equation](http://latex.codecogs.com/gif.latex?h^{'}(x_t)=[0,0,0,0,0,0,1])

### Implementation

Fine tunning the parameter ``QYawStd`` in ``QuadEstimatorEKF.txt``, when the value is less than
0.05, the white bound is too narrow; while larger than 0.05, it chases the trend of magnitude of
the drift.

``QYawStd`` in ``QuadEstimatorEKF.txt``:

```
QYawStd = .07
```

If QYawStd > 0.05 (e.g. 0.07), the drift is between the both white lines:

![10_MagUpdate_tuned.png](./images/10_MagUpdate_tuned.png)

Codes implemented in ``UpdateFromMag()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   hPrime(0, 6) = 1;

   zFromX(0) = ekfState(6);
   float diffZ = z(0) - zFromX(0);
   if (diffZ > F_PI) { zFromX(0) += 2.f * F_PI; }
   else if (diffZ < -F_PI) { zFromX(0) -= 2.f * F_PI; }
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Run the result:

```
Simulation #1 (../config/10_MagUpdate.txt)
Simulation #2 (../config/10_MagUpdate.txt)
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 64% of the time
```

The chart illustrates the estimated standard deviation capturing the error and maintaining
it in the bound (-0.1, 0.1).

![10_MagUpdate](./images/10_MagUpdate.png)


## Solution: Scenario 11_GPSUpdate

The GPS update phase of EKF is as the same as the Magnetometer except the input values in
``UpdateFromGPS()``. The coresponding values are as below:

- ``z``: the input the position(x, y, z) and velocity(x, y, z);
- ``R_GPS``: the GPS measurement covariance;
- ``zFromX``: the predicted position(x, y, z) and velocity(x, y, z) in ekfState;
- ``hPrime``:

![equation](./images/gps_hprime.gif)

### Implementation

**Ideal Estimator + Ideal IMU**

Choose the scenario 11_GPSUpdate, the running scenario, using the ideal esitmator and ideal
IMU, is illustrated as below. We could see the chart at the bottom right that position error
and Z std are drifting away because of the incomplete GPS update function.

![11_GPSUpdate_ideal](./images/11_GPSUpdate_ideal.png)

**Realistic Estimator + Ideal IMU**

Updating ``Quad.UseIdealEstimator`` in ``config/11_GPSUpdate.txt`` from 1 to 0 to switch the
Estimator from ideal to realistic.

```
# Quad.UseIdealEstimator:
# - 0: realistic
# - 1: ideal
Quad.UseIdealEstimator = 0
```

Differentiating from the ideal estimator using the ideal IMU, the shape of path keeps approximately
the same but the significant deviations of global positions occurred if the realistic estimator using
the ideal IMU, as the result shown at the below image. 

![11_GPSUpdate_realsitic](./images/11_GPSUpdate_realistic.png)

**Realistic Estimator + Realistic IMU**

Commented the IMU settings in ``config/11_GPSUpdate.txt``:

```
# SimIMU.AccelStd = 0,0,0
# SimIMU.GyroStd = 0,0,0
```

The path is completely incorrect. As the charts illustrated, the deviations between the true position y
and velocity y and the estimated position y and estimated velocity vy are shown in the first graph. Meanwhile,
the position error is significantly increased up to 30-40.

![11_GPSUpdate_realistic_imu](./images/11_GPSUpdate_realistic_imu.png)

Tuning the parameter ``QPosZStd`` in ``config/QuadEstimatorEKF.txt``:

```
QPosZStd = .1
```

If the standard deviation increases, the errors of the positions will increased corespondingly. The smaller
the standard deviation is, the more approximate the expected results will meet.

![11_GPSUpdate_z_0.1](./images/11_GPSUpdate_z_0.1.png)

Codes implemented in ``UpdateFromGPS()``:

```
   ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   hPrime(0, 0) = 1;
   hPrime(1, 1) = 1;
   hPrime(2, 2) = 1;
   hPrime(3, 3) = 1;
   hPrime(4, 4) = 1;
   hPrime(5, 5) = 1;

   zFromX(0) = ekfState(0);
   zFromX(1) = ekfState(1);
   zFromX(2) = ekfState(2);
   zFromX(3) = ekfState(3);
   zFromX(4) = ekfState(4);
   zFromX(5) = ekfState(5);
   /////////////////////////////// END STUDENT CODE ////////////////////////////
```

Run the result:

```
Simulation #1 (../config/11_GPSUpdate.txt)
Simulation #2 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```

With the update of the GPS measurement, the green box shows the result meets the criteria that
the estimated position error is less than 1m for at least 20 seconds.

![11_GPSUpdate](./images/11_GPSUpdate.png)

## Solution: Scenario 11_GPSUpdate (Control Integration)

Copied the ``QuadController.cpp`` from last project, and update its parameters in ``config/QuadControlParams``,
de-tuned the parameters as

```
# Position control gains
kpPosXY = 3
kpPosZ = 4
KiPosZ = 20

# Velocity control gains
kpVelXY = 10
kpVelZ = 10

# Angle control gains
kpBank = 12
kpYaw = 4

# Angle rate gains
kpPQR = 60, 60, 15
```

Run the result:

```
Simulation #47 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```

The chart is shown as

![11_GPSUpdate_control](./images/11_GPSUpdate_control.png)
