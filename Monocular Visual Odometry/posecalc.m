base_dir = '\\engin-labs.m.storage.umich.edu\pwestra\windat.v2\Desktop\Monocular Visual Odometry';
oxts = loadOxtsliteData(base_dir);
pose = convertOxtsToPose(oxts);
counter = 1;