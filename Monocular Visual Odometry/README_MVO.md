# README - Monocular Visual Odometry
This example provides relative SE3 transformations for first 302 poses.
1. Download folder named Monocular Visual Odometry.
2. Run MATLAB script `posecalc_toy_example.m`; you can, as an alternative, run the function `posecalc.m` giving the directory of the `posecalc` function and desired frames (i.e. 1:302) as arguments. 
3. Open the script `Data_Edit`, ensure the variable “starter” in line 2 of the code is set to a value of 1 and line 20 of `Data_Edit` points towards the file named image01.
4. Open `VisualOdometryExample_2.m` and run the code.
5. Go back into the `Data_Edit.m` function and change the variable “starter” in line 2 of the code to a value of 102.
6. Change the directory in line 20 of `Data_Edit` to point towards the file named image02 within the monocular visual odometry folder.
7. Run the `VisualOdometryExample_2.m`.
8. Go back into the `Data_Edit.m` function and change the variable “starter” in line 2 of the code to a value of 202.
9. Change the directory in line 20 of `Data_Edit` to point towards the file named image03 within the monocular visual odometry folder.
10. Run the `VisualOdometryExample_2.m`. 
11. The variable Relative_total contains the relative SE3 transformations between the first 302 poses of the trajectory. And is used in the factor graph construction.

