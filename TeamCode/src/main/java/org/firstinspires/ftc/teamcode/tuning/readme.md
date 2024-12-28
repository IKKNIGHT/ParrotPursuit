# Tuning


## Otos

### Angular Scalar Tuner

To tune the `OTOSAngularScalarTuner`, run the opmode and follow the directions in telemetry and then take your value and dump it in the variable located
in `DriveConstants.SparkFunOTOSConfig.ANGULAR_SCALAR`.

### Linear Scalar Tuner

To tune the Linear Scalar in the `DriveConstants.SparkFunOTOSConfig.LINEAR_SCALAR`, you must follow the following steps.

move the robot a known distance and measure the error; do this multiple times at multiple speeds to get an average, then set the linear scalar to the
inverse of the error. For example, if you move the robot 100 inches and the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971.

### Position Offset Tuner

to tune the `OTOSPositionOffsetTuner`, run the opmode and follow its directions in the telemetry, then take your value and dump it in the x,y section of
the `DriveConstants.SparkFunOTOSCondig.OFFSET.x/y` the format of the Pose2d in the Drive Constants is (x,y,h) where x and y are the offsets you were given
via the tuning opmode.

### Heading Offset Tuner

to tune the `OTOSHeadingOffsetTuner`, run the opmode and follow its directions from the telemetry, then take your value and dump it in the h section of 
the `DriveConstants.SparkFunOTOSConfig.ANGULAR_SCALAR` the format of the Pose2d in the Drive Constants is (x,y,h) where h is the offsets you were given
via the tuning opmode.