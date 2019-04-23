# README - Monocular Visual Odometry
This example provides relative SE3 transformations for first 302 poses.
1. Download folder named Monocular Visual Odometry.
2. Run function posecalc.m.
3. Open the function Data_Edit and change the path directory in line 20 of Data_Edit to point towards the file named image01 within the monocular visual odometry folder.
4. Open VisualOdometryExample_2.m and run the code.
5. Go back into the Data_Edit.m function and change the variable “starter” in line 2 of the code to a value of 102.
6. Change the directory in line 20 of Data_Edit to point towards the file named image02 within the monocular visual odometry folder.
7. Run the VisualOdometryExample_2.m.
8. Go back into the Data_Edit.m function and change the variable “starter” in line 2 of the code to a value of 202.
9. Change the directory in line 20 of Data_Edit to point towards the file named image03 within the monocular visual odometry folder.
10. The variable Relative_total contains the relative SE3 transformations between the first 302 poses of the trajectory. And is used in the factor graph construction.

