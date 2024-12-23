package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.geometry.Twist2d;

public class ThreeDeadWheelOdometry extends Localizer{
     DcMotor leftEncoder = null;
     DcMotor rightEncoder = null;
     DcMotor horizontalEncoder = null;
    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;

    public Pose2d robotPose;
    public Pose2d previousPose;
    long start_time = System.nanoTime();


    /**
     * Constructs a {@code ThreeDeadWheelOdometry} with a default starting pose of (0, 0, 0).
     *
     */
    public ThreeDeadWheelOdometry(){
        this(new Pose2d());
    }
    /**
     * Constructs a {@code ThreeDeadWheelOdometry} with a default starting pose of (0, 0, 0).
     *
     * @param initialPose the initial pose estimate for the robot.
     */
    public ThreeDeadWheelOdometry(Pose2d initialPose){
        super();
        previousAngle = initialPose.getRotation();
        robotPose = initialPose;
        previousPose = initialPose;
    }
    /**
     * Constructs a {@code ThreeDeadWheelOdometry} with a hardwareMap for retrieving the encoders.
     *
     * @param hwMap the hardware map used to retrieve the {@code ThreeDeadWheelOdometry}.
     */
    public ThreeDeadWheelOdometry(HardwareMap hwMap){
        this();
        leftEncoder = hwMap.get(DcMotor.class, DriveConstants.ThreeDeadWheelConfig.LEFT_ENCODER);
        rightEncoder = hwMap.get(DcMotor.class, DriveConstants.ThreeDeadWheelConfig.RIGHT_ENCODER);
        horizontalEncoder = hwMap.get(DcMotor.class, DriveConstants.ThreeDeadWheelConfig.HORIZONTAL_ENCODER);
    }
    @Override
    public Pose2d update() {
        update(leftEncoder.getCurrentPosition(),rightEncoder.getCurrentPosition(),horizontalEncoder.getCurrentPosition());
        return robotPose;
    }

    @Override
    public void setPoseEstimate(Pose2d newPose) {
        previousAngle = newPose.getRotation();
        previousPose = robotPose;
        robotPose = newPose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return robotPose;
    }

    @Override
    public Transform2d getVelocity() {
        return new Transform2d(new Pose2d(),new Pose2d(getXVelocity(),getYVelocity(),getThetaVelocity()));
    }
    /**
     * updates the robot's pose estimate based on encoder readings and returns the updated pose.
     *
     * @param leftEncoderPos the current position of the left encoder.
     * @param rightEncoderPos the current position of the right encoder.
     * @param horizontalEncoderPos the current position of the horizontal encoder.
     */
    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                new Rotation2d(
                        (deltaLeftEncoder - deltaRightEncoder) / DriveConstants.ThreeDeadWheelConfig.TRACK_WIDTH
                )
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (DriveConstants.ThreeDeadWheelConfig.CENTER_WHEEL_OFFSET * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;
        previousPose = robotPose;
        start_time = System.nanoTime(); // reset time here

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
    /**
     * gets the current X Velocity of the robot.
     * @return the current X Velocity of the robot.
     */
    public double getXVelocity(){
        return (previousPose.getX()-robotPose.getX())/(System.nanoTime()-start_time);
    }
    /**
     * gets the current Y Velocity of the robot.
     * @return the current Y Velocity of the robot.
     */
    public double getYVelocity(){
        return (previousPose.getY()-robotPose.getY())/(System.nanoTime()-start_time);
    }
    /**
     * gets the current Heading Velocity of the robot.
     * @return the current Heading Velocity of the robot.
     */
    public Rotation2d getThetaVelocity(){
        return new Rotation2d((previousAngle.getRadians()-robotPose.getRotation().getRadians())/(System.nanoTime()-start_time));
    }

}
