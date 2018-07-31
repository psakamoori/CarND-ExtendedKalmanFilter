# CarND-ExtendedKalmanFilter
Udacity - Term2 - Project 1

# Overview

This project consists of C++ implentation of Extended Kalam Filter as part of Udacity Self-Driving Car Nanodegree - Extended Kalman Filter project (Term2). Based on LIDAR and RADAR data as input, we use EKF to Predict and Update Position and Velocity information of the Car. A simulator is provided to test on two different Data Sets. Communication between simulator and EKF is done using WebSocket.

# Prerquisites

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator

For detailed instructions on how to install ref: https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

# Compiling and executing the project

- Clone the repo and cd to the project using Ubuntu Terminal
- cd to build 
- Run 'cmake ..." and "make" commands
- Above commands will create "ExtendedKF" executable

# Testing
- From build directory, execute .'/ExtendedKF'. Output should be:

```
Listening to port 4567
Connected!!!
```

- Once you see above output, on simulator select "Dataset 1/2" and press "start" button.
- You should see RMSE value on right side on the Simulator window

# Accuracy
- As stated RMSE value (px, py, vx and vy) should be <= [.11, .11, 0.52, 0.52]

# Output

- DataSet1 = [0.09, 0.08, 0.45, 0.43]
- DataSet2 = [0.07, 0.09, 0.42, 0.49]
