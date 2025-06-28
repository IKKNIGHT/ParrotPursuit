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
        // ============ BASIC DRIVE PARAMETERS ============
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
        
        // ============ PURE PURSUIT CORE PARAMETERS ============
        /**
         * Primary lookahead distance in inches. This is the most important tuning parameter.
         * - Too small (< 6"): Robot will oscillate and be unstable
         * - Too large (> 24"): Robot will cut corners and overshoot
         * - Recommended starting point: 12-15 inches for most FTC robots
         */
        public static double LOOKAHEAD_DISTANCE = 12.0;
        
        /**
         * Minimum lookahead distance in inches. Robot will never look closer than this.
         * Prevents instability at low speeds. Recommended: 4-8 inches.
         */
        public static double MIN_LOOKAHEAD_DISTANCE = 6.0;
        
        /**
         * Maximum lookahead distance in inches. Prevents over-cutting corners at high speeds.
         * Recommended: 18-30 inches depending on field size.
         */
        public static double MAX_LOOKAHEAD_DISTANCE = 24.0;
        
        /**
         * Distance tolerance in inches for path completion detection.
         * Robot stops when within this distance of the path end.
         * Recommended: 1-3 inches for precision, 3-6 inches for speed.
         */
        public static double PATH_COMPLETION_TOLERANCE = 2.0;
        
        // ============ VELOCITY CONTROL PARAMETERS ============
        /**
         * Maximum robot velocity in inches per second.
         * Should match your robot's actual maximum safe speed.
         * Typical FTC values: 20-60 inches/second
         */
        public static double MAX_VELOCITY = 24.0;
        
        /**
         * Minimum robot velocity in inches per second.
         * Robot will never go slower than this (prevents stalling).
         * Recommended: 3-8 inches/second
         */
        public static double MIN_VELOCITY = 6.0;
        
        /**
         * How much to slow down for high curvature (sharp turns).
         * Range: 0.0 (no slowdown) to 1.0 (maximum slowdown)
         * - 0.3 = gentle slowdown for smooth paths
         * - 0.7 = aggressive slowdown for tight corners
         * Recommended starting point: 0.5
         */
        public static double CURVATURE_VELOCITY_SCALING = 0.5;
        
        // ============ ADVANCED TUNING PARAMETERS ============
        /**
         * Path sampling resolution for finding closest points.
         * Smaller values = more precise but slower computation.
         * Range: 0.001 to 0.05. Recommended: 0.01
         */
        public static double PATH_RESOLUTION = 0.01;
        
        /**
         * How aggressively to adapt lookahead distance based on velocity.
         * Range: 0.0 (no adaptation) to 1.0 (full adaptation)
         * Recommended: 0.3 for most applications
         */
        public static double ADAPTIVE_LOOKAHEAD_GAIN = 0.3;
        
        /**
         * Maximum allowed curvature (1/radius) to prevent impossible turns.
         * Higher values = tighter turns allowed.
         * Recommended: 0.1 to 0.5 depending on robot maneuverability
         */
        public static double MAX_CURVATURE = 0.2;
        
        /**
         * Smoothing factor for velocity changes (0.0 to 1.0).
         * Lower values = smoother acceleration/deceleration
         * Higher values = more responsive but jerkier motion
         * Recommended: 0.1 to 0.3
         */
        public static double VELOCITY_SMOOTHING = 0.2;
        
        /**
         * Distance ahead to start slowing down for path completion.
         * Should be larger than PATH_COMPLETION_TOLERANCE.
         * Recommended: 2-4 times the completion tolerance
         */
        public static double DECELERATION_DISTANCE = 8.0;
        
        /**
         * Minimum power output to motors (prevents stalling).
         * Range: 0.0 to 0.3. Recommended: 0.05 to 0.1
         */
        public static double MIN_MOTOR_POWER = 0.05;
        
        /**
         * Maximum angular velocity in radians per second.
         * Prevents excessive spinning. Typical values: 2-8 rad/s
         */
        public static double MAX_ANGULAR_VELOCITY = 4.0;
        
        // ============ SAFETY AND PERFORMANCE PARAMETERS ============
        /**
         * Timeout for path following in milliseconds.
         * Robot will stop if path takes longer than this.
         * Set to 0 to disable timeout. Recommended: 15-30 seconds
         */
        public static long PATH_TIMEOUT_MS = 20000;
        
        /**
         * Maximum acceleration in inches/second^2.
         * Prevents jerky motion and wheel slipping.
         * Recommended: 24-60 for most FTC robots
         */
        public static double MAX_ACCELERATION = 36.0;
        
        /**
         * Control loop frequency in Hz.
         * Higher = more responsive but more CPU intensive.
         * Recommended: 20-50 Hz for FTC
         */
        public static double CONTROL_FREQUENCY = 50.0;
    }
}
