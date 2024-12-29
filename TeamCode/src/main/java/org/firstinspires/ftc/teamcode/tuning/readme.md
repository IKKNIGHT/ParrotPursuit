# Tuning

For tuning, please tune your odometry module (SparkFunOtos, pinpoint, 3 DeadWheel Odometry)
then tune the PID's, and track width.

In essence the only class you will touch is the `DriveConstants` class as everything you will need to change will be in there.

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

## 3 Dead wheel Odometry

### Encoder Config

for 3 deadwheel odometry put in the names of your encoders as stated in the config file in `DriveConstants.ThreeDeadWheelConfig` there substitute for
`left_encoder`, `right_encoder`, `horizontal_encoder`

### Track width

Not to be confused as the track width in `DriveConstants.TunableParams`; as of right now, find the distance between the parallel dead wheels in inches
and put in the value into `DriveConstants.ThreeDeadWheelConfig.TRACK_WIDTH`

### Center Wheel Offset
The center wheel offset accounts for the distance between the center of rotation of the robot and the position of the horizontal encoder.
Find this value and dump it in the `DriveConstants.ThreeDeadWheelConfig.CENTER_WHEEL_OFFSET`.

## Pinpoint Odometry

### Config
Take the config name of the pinpoint odo in the Driver station and copy it in the `DriveConstants.PinpointConfig.PINPOINT_NAME`.

### Offsets
Set the odometry pod positions relative to the point that the odometry computer tracks around.
The X pod offset refers to how far sideways from the tracking point the
X (forward) odometry pod is. Left of the center is a positive number,
right of center is a negative number. the Y pod offset refers to how far forwards from
the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
backwards is a negative number.

The X value is going to be dumped in `DriveConstants.PinpointConfig.PINPOINT_X_OFFSET`
The Y value is going to be dumped in `DriveConstants.PinpointConfig.PINPOINT_Y_OFFSET`

### Directions
Set the direction that each of the two odometry pods count. The X (forward) pod should
increase when you move the robot forward. And the Y (strafe) pod should increase when
you move the robot to the left.

Dump these values in `DriveConstants.PinpointConfig.X_DIRECTION` and `DriveConstants.PinpointConfig.Y_DIRECTION` respectively

### Gobilda Pod Type / Encoder Resolution
Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.

Dump this value in `DriveConstants.PinpointConfig.POD`

If you aren't using goBilda odometry pods, Go to the `localization.PinpointLocalizer` class; in line 36 replace the line with
`odo.setEncoderResolution(13.26291192);` and replace the number with  the number of ticks per mm of your odometry pod.

 