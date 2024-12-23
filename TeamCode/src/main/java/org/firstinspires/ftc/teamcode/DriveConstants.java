package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;

public class DriveConstants {
    // TODO: change the localiser type, to a class that extends localizer
    //   ex : public static final Class LOCALIZER_CLASS = ThreeDeadWheelOdometry.class;
    public static final Class LOCALIZER_CLASS = PinpointLocalizer.class;
    // public static final Class LOCALIZER_CLASS = ThreeDeadWheelOdometry.class;


    public static class ThreeDeadWheelConfig{
        public static final String LEFT_ENCODER = "left_encoder";
        public static final String RIGHT_ENCODER = "right_encoder";
        public static final String HORIZONTAL_ENCODER = "horizontal_encoder";
        public static final double CENTER_WHEEL_OFFSET = 1.0;
        public static final double TRACK_WIDTH = 10.0;
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
    public static class TuneableParams {
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
