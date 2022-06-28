# MOVEL-TEST

### A brief documentation on what prerequisite things need to be checked (hardware and software checks) before starting to do a mapping or navigation task

Hardware <br>
-----
- [ ] Power Distribution Board
- [ ] Sensor
  - [ ] Camera 
  - [ ] Inertial Measurment Unit (IMU)
  - [ ] Odometer
  - [ ] Lidar
- [ ] Robot's Model
  - This is so urgent too for analytical solution in example for calculating control system (ex : LQR, LQG, Model Predictive Control)
- [ ] Computer
- [ ] Sensor Module Board
  - Some sensor doesn't have serial port, so we have to made sensor module (example for IMU and Odometer) 

Software <br>
-----
- [ ] Framework
  - This urgently for controling all thread, and fast deploying
- [ ] Module
  - [ ] Filtering Sensor
    - Can be used sampling method for reduce dimension of the lidar sensor
    - Must be concern on odometer result, data may lead messy pose estimation
    - Can be used filtering method (example KF, EKF) for filtering the IMU sensor
  - [ ] Feature Extraction from Camera
    - For additional feature on Pose Estimation using Monte-Carlo or Adaptive Monte Carlo
  - [ ] Pose Estimation
    - Can be used Monte Carlo Method or Adaptive Monte Carlo
  - [ ] Path Planning
    - Must be concern with memmory usaged
  - [ ] Interface
    - This is simple but so urgent for monitoring
- [ ] Computer (Must be handled all proccessing)

-----
# How to run
Create workspace folder

```
mkdir -p ~/movel_ws/src
cd ~/movel_ws/src
```
Clone project
```
git clone https://github.com/Mikael17125/movel-test.git .
```
Build image container
```
cd ~/movel_ws/src/path_generator/
docker build -t path-generator:1.0 .
```
```
cd ~/movel_ws/src/rviz_visualizer/
docker build -t rviz-visualizer:1.0 .
```
```
cd ~/movel_ws/src/path_reduction/
docker build -t path-reduction:1.0 .
```
Start Program
```
docker-compose up -d
```

# Documention and Parameter
-----
<h1>ENV VARIABLE</h1>
To input point of path reduce: this will path to be N point <br>
"N_POINT=10" <br>
This is to select the spline method 0 : Lagrange Method; 1 : Cubic Spline Method <br>
"SPLINE=0" <br>
This is to select the spline coordinate <br>
"COORD=1" <br>

The coordinate is :

```python
dt = np.array([0,1,2,3,4])
x = np.array([-3, 2, -2, -1, 1])
y = np.array([3, 2,  1, -1, 5])

dt1 = np.array([0,1,2,3,4])
x1 = np.array([0, 4, -2, 1, 0])
y1 = np.array([1, 2,  -4, 1, -3])
```

# Explanation
- [ ] Before I'm started explained why I'm choose Cubic Spline and Lagrange Interpolation, I have read some article, there is another method like bezier curve or B-Spline and many Interpolation method, and then I'm choose the Cubic Spline because this simplicity of calculation, numerical stability and smoothness of the interpolated curve and for disadvantages is the continuous power flow is that it requires a considerable amount of time and hence it cannot be employed in real-time applications and the second method is interpolation, this advantages of Lagrange Interpolation is will be more accurate for higher order polynominals, and for advantages is this method used greedy solution and will be tedious job to do when the polynominal order increase because the number of point increases for realtine case
- [ ] For the 2nd Case I'm using sampling method, first I'm filter the path and then filtered path will be reduce with sampling method there is three color on the simulation, green color represented original path, red color represented filtered path and blue color represented reduces path, this is simplest method can be used but, this program spend many resources

# VIDEO LINK
-----
[Youtube : Video Documentation Link](https://www.youtube.com/watch?v=q5Tb2qVTZEU)
