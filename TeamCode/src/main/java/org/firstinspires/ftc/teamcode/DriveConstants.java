package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;

public class DriveConstants {
    // TODO: change the localizer type, to a class that extends localizer
    //   ex : public static final Class LOCALIZER_CLASS = ThreeDeadWheelLocalizer.class;
    public static final Class LOCALIZER_CLASS = PinpointLocalizer.class;
    // public static final Class LOCALIZER_CLASS = ThreeDeadWheelOdometry.class;
    // public static final Class LOCALIZER_CLASS = SparkfunOTOSLocalizer.class;


    // TODO: Configure one of these Localization Configs
    public static class PinpointConfig{
        public static String PINPOINT_NAME="odo";
        public static double PINPOINT_X_OFFSET=0;
        public static double PINPOINT_Y_OFFSET=0;
        public static GoBildaPinpointDriver.GoBildaOdometryPods POD = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public static GoBildaPinpointDriver.EncoderDirection X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static GoBildaPinpointDriver.EncoderDirection Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
    public static class ThreeDeadWheelConfig{
        public static final String LEFT_ENCODER = "left_encoder";
        public static final String RIGHT_ENCODER = "right_encoder";
        public static final String HORIZONTAL_ENCODER = "horizontal_encoder";
        public static final double CENTER_WHEEL_OFFSET = 1.0;
        public static final double TRACK_WIDTH = 10.0;
    }
    public static class SparkFunOTOSConfig {
        public static final String OTOS_NAME = "otos";

        /* Assuming you've mounted your sensor to a robot and it's not centered,
         you can specify the offset for the sensor relative to the center of the
         robot. The units default to inches and degrees, but if you want to use
         different units, specify them before setting the offset! Note that as of
         firmware version 1.0, these values will be lost after a power cycle, so
         you will need to set them each time you power up the sensor. For example, if
         the sensor is mounted 5 inches to the left (negative X) and 10 inches
         forward (positive Y) of the center of the robot, and mounted 90 degrees
         clockwise (negative rotation) from the robot's orientation, the offset
         would be {-5, 10, -90}. These can be any value, even the angle can be
         tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).*/

        public static final SparkFunOTOS.Pose2D OFFSET = new SparkFunOTOS.Pose2D(0, 0, 0);

        /* Here we can set the linear and angular scalars, which can compensate for
         scaling issues with the sensor measurements. Note that as of firmware
         version 1.0, these values will be lost after a power cycle, so you will
         need to set them each time you power up the sensor. They can be any value
         from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
         first set both scalars to 1.0, then calibrate the angular scalar, then
         the linear scalar. To calibrate the angular scalar, spin the robot by
         multiple rotations (eg. 10) to get a precise error, then set the scalar
         to the inverse of the error. Remember that the angle wraps from -180 to
         180 degrees, so for example, if after 10 rotations counterclockwise
         (positive rotation), the sensor reports -15 degrees, the required scalar
         would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
         robot a known distance and measure the error; do this multiple times at
         multiple speeds to get an average, then set the linear scalar to the
         inverse of the error. For example, if you move the robot 100 inches and
         the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971*/
        public static final double LINEAR_SCALAR = 1.0;
        public static final double ANGULAR_SCALAR = 1.0;
    }
    public static class MotorConfig{
        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        public static final String LEFT_FRONT = "frontleft";
        public static final String RIGHT_FRONT = "frontright";
        public static final String LEFT_BACK = "backleft";
        public static final String RIGHT_BACK = "backright";
    }
    /**
     * Parameters for controlling the robot, including feedforward and PID coefficients.
     */
    public static class TunableParams {
        public static double FORWARD_KS = 0;
        public static double FORWARD_KV = 1;

        public static double STRAFE_KS = 0;
        public static double STRAFE_KV = 1;

        public static double ROTATIONAL_KS = 0;
        public static double ROTATIONAL_KV = 1;

        public static double TRANSLATIONAL_KP = 0.01;
        public static double TRANSLATIONAL_KD = 0.01;

        public static double HEADING_KP = 0.01;
        public static double HEADING_KD = 0.01;
    }
}
