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

    public Pose2d robotPose = new Pose2d(0,0,new Rotation2d(0)); // 0,0,0
    long start_time = System.nanoTime();



    public ThreeDeadWheelOdometry(){
        this(new Pose2d());
    }
    public ThreeDeadWheelOdometry(Pose2d initialPose){
        super();
        previousAngle = initialPose.getRotation();
    }
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
        return new Transform2d(new Pose2d(),new Pose2d());
    }
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
        start_time = System.nanoTime(); // reset time here

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
}
