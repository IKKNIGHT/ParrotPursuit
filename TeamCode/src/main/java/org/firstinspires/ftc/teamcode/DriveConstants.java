package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;

public class DriveConstants {
    // TODO: change the localizer type, to a class that extends localizer
    //   ex : public static final Class LOCALIZER_CLASS = ThreeDeadWheelLocalizer.class;
    public static final Class LOCALIZER_CLASS = PinpointLocalizer.class;
    // public static final Class LOCALIZER_CLASS = ThreeDeadWheelLocalizer.class;
    // public static final Class LOCALIZER_CLASS = SparkFunOTOSLocalizer.class;


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
        public static final double CENTER_WHEEL_OFFSET = 1.0; // the
        public static final double TRACK_WIDTH = 10.0; // the distance between the left and right wheels
    }
    public static class SparkFunOTOSConfig {
        public static final String OTOS_NAME = "otos";
        public static final SparkFunOTOS.Pose2D OFFSET = new SparkFunOTOS.Pose2D(0, 0, 0); // from tuning copy all ur offsets in the respective positions here. x,y,heading
        public static final double LINEAR_SCALAR = 1.0; // You must tune this yourself... Tuning instructions in tuning.readme.md
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
        // TODO: tune these coefficients
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

        public static double TRACK_WIDTH = 10.0;
        
        // Pure Pursuit Parameters
        public static double LOOKAHEAD_DISTANCE = 12.0; // inches - distance to look ahead on the path
        public static double MIN_LOOKAHEAD_DISTANCE = 6.0; // minimum lookahead distance
        public static double MAX_LOOKAHEAD_DISTANCE = 24.0; // maximum lookahead distance
        public static double PATH_COMPLETION_TOLERANCE = 2.0; // inches - how close to end of path to consider complete
        public static double MAX_VELOCITY = 24.0; // inches per second - maximum robot velocity
        public static double MIN_VELOCITY = 6.0; // inches per second - minimum robot velocity
        public static double CURVATURE_VELOCITY_SCALING = 0.5; // how much to slow down for high curvature
    }
}
