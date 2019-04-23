# Read me

This is a instruction about how to run the ISAM2 backend program. The only dependency -- gtsam MATLAB wrapper is included in the folder as gtsam_toolbox.zip. 

##### Step 1:

Unpack the gtsam MATLAB wrapper -- gtsam_toolbox.zip and the sample observation -- ovservation.zip

##### Step 2:

Use the MATLAB to open the script run.m

##### Step 3:

Add the gtsam MATLAB wrapper to your workspace by right click the unpacked grsam_toolbox folder and select Add to Path -> Selected folder

##### Step 4:

You can test the toolbox by running the toy examples I prepared. To run the toy examples be sure that two lines about ISAM2_back_end are all comment out. 

##### Step 5:

To run the ISAM2 backend program, a path to the observation should be added by setting the global variable measurement_path in run.m.