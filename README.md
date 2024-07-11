# CBF example - Double Integeral 2D



### Introduction
---
This project implements Control Barrier Functions (CBFs) in MATLAB for a double integrator system. 
The CBF is used to calculate the control input while ensuring the system's safety. 
The project includes a base CBF controller class and a derived class for the double integrator system.

<br/>

### Folder Structure
---
|--- doubleIntegral2D_main.m

|--- classes/

|---  |--- base/  CBFcontroller.m

|---  |--- derived/ CBFdoubleIntegral2D.m

<br/>

### Example Results
---
#### Position vs Time & Velocity vs Time
<img src = "https://github.com/yeongdg/CBF-doubleIntegeral2D/blob/main/img/time_PosVel_doubleIntegral2D.jpg">

#### Phase Plot (Position vs Velocity)
<img src = "https://github.com/yeongdg/CBF-doubleIntegeral2D/blob/main/img/pos_vel_doubleIntegral2D.jpg">

<br/>

### Citing
---
* F. Ferraguti et al., "Safety and Efficiency in Robotics: The Control Barrier Functions Approach," in IEEE Robotics & Automation Magazine, vol. 29, no. 3, pp. 139-151, Sept. 2022, doi: 10.1109/MRA.2022.3174699. [[IEEE]](https://ieeexplore.ieee.org/abstract/document/9788028)
