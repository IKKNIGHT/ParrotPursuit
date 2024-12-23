package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.controllers.SimpleMotorFeedforward;

/**
 * The {@code MecanumDrive} class implements the control logic for a mecanum wheel drivetrain.
 * It provides functionality to control motor powers directly and weighted power calculations
 * for smooth movements in different directions.
 */
public class MecanumDrive extends Follower{

    // Motor objects for the mecanum drivetrain
    DcMotorEx leftFront, leftBack, rightBack, rightFront;

    // Feedforward controllers for each axis of motion
    SimpleMotorFeedforward forwardFeedforward, strafeFeedforward, headingFeedforward;



    /**
     * Constructs a {@code MecanumDrive} object.
     *
     * @param hardwareMap the hardware map used to retrieve motor and sensor configurations.
     * @param startPose        the starting pose of the robot.
     */
    public MecanumDrive(HardwareMap hardwareMap, Pose2d startPose) {
        super(hardwareMap, startPose);

        // Initialize motors based on hardware configuration
        leftFront = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.RIGHT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.RIGHT_FRONT);

        // Set motors to brake when zero power is applied
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Configure motor direction if needed (currently commented out)
        // leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize feedforward controllers
        forwardFeedforward = new SimpleMotorFeedforward(DriveConstants.TunableParams.FORWARD_KS, DriveConstants.TunableParams.FORWARD_KV);
        strafeFeedforward = new SimpleMotorFeedforward(DriveConstants.TunableParams.STRAFE_KS, DriveConstants.TunableParams.STRAFE_KV);
        headingFeedforward = new SimpleMotorFeedforward(DriveConstants.TunableParams.ROTATIONAL_KS, DriveConstants.TunableParams.ROTATIONAL_KV);
    }

    /**
     * Sets raw motor powers for the drivetrain.
     * If any power exceeds 1, all powers are scaled proportionally to maintain ratios.
     *
     * @param frontLeft  power for the front left motor.
     * @param frontRight power for the front right motor.
     * @param backLeft   power for the back left motor.
     * @param backRight  power for the back right motor.
     */
    public void setRawPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        // Normalize motor powers if any value exceeds 1
        double maximum = Math.max(Math.max(frontLeft, frontRight), Math.max(backLeft, backRight));
        if (maximum > 1) {
            frontLeft /= maximum;
            frontRight /= maximum;
            backLeft /= maximum;
            backRight /= maximum;
        }

        // Apply powers to motors
        leftFront.setPower(frontLeft);
        leftBack.setPower(backLeft);
        rightFront.setPower(frontRight);
        rightBack.setPower(backRight);
    }

    /**
     * Sets motor powers based on weighted feedforward values for forward, strafe, and heading movements.
     *
     * @param front   forward movement power.
     * @param strafe  strafe (sideways) movement power.
     * @param heading rotational movement power.
     */
    @Override
    public void setWeightedPowers(double front, double strafe, double heading) {
        // Calculate weighted feedforward values
        double weightedFront = forwardFeedforward.calculate(front);
        double weightedStrafe = strafeFeedforward.calculate(strafe);
        double weightedHeading = headingFeedforward.calculate(heading);

        // Calculate and set motor powers
        setRawPowers(
                (weightedFront - weightedStrafe - weightedHeading),
                (weightedFront + weightedStrafe + weightedHeading),
                (weightedFront + weightedStrafe - weightedHeading),
                (weightedFront - weightedStrafe + weightedHeading)
        );
    }
}
