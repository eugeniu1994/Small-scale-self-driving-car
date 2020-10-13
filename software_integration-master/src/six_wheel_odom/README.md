# rs_cam_and_imu_filter_launch

This is a node that recieves SixWheelInfo messages and publishes nav_msgs/odometry. Note that due to many harware based arrors on 6 wheeled car it usually fails. Note that its sparam file is not in use and not true. Unfortunatly gain are calcuated inside the code maybe a good step would be create a param file to set gains.