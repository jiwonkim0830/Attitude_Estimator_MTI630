# Attitude_Estimator_MTI630
Attitude estimator using MTI630 (Mahony filter, EKF)


## How to use 

1. Download MTI software (https://www.xsens.com/software-downloads)
2. With mtmanager, do intial setting
    - to use mtmanager you have install some dependencies based on MTM.README file.
    - have to run with sudo
3. cd [downloaed file] && ./mtsdk_[os]_[version].sh  to get sdk 
    - prerequisite : sharutils
    - you can check MTSDK.README file in sdk directory

4. cd [sdk dir]/examples/xda_cpp  (you can edit directory as you want)
5. clone this repository
    - prerequisite : libtf2-ros-dev
    - have to run with sudo
6. edit cmake file as you want

7. choose the filter on main.cpp & Initialize_IMU.hpp (edit #define on top)

## Checks

1. Filters are initialized as initial orientation is zero orientation. :  initial quaterion(w,x,y,z) = (1,0,0,0)


## Reference

1. R. Mahony, T. Hamel and J. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group," in IEEE Transactions on Automatic Control, vol. 53, no. 5, pp. 1203-1218, June 2008, doi: 10.1109/TAC.2008.923738.
2. A. M. Sabatini, "Quaternion-based extended Kalman filter for determining orientation by inertial and magnetic sensing," in IEEE Transactions on Biomedical Engineering, vol. 53, no. 7, pp. 1346-1356, July 2006, doi: 10.1109/TBME.2006.875664.