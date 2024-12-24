package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Transform2d;

public class SparkFunOTOSLocalizer extends Localizer {
    SparkFunOTOS otos;
    /**
     * Constructs a {@code SparkfunOTOSLocalizer} with a default starting pose of (0, 0, 0).
     *
     * @param hardwareMap the hardware map used to retrieve the {@code SparkFunOTOS}.
     */
    public SparkFunOTOSLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d());
    }

    /**
     * Constructs a {@code SparkfunOTOSLocalizer} with a specified starting pose.
     * @param hardwareMap the hardware map used to retrieve the {@code SparkFunOTOS}.
     * @param startPose the initial pose estimate for the robot.
     */
    public SparkFunOTOSLocalizer(HardwareMap hardwareMap, Pose2d startPose){
        otos = hardwareMap.get(SparkFunOTOS.class, DriveConstants.SparkFunOTOSConfig.OTOS_NAME);
        otos.resetTracking();
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setOffset(DriveConstants.SparkFunOTOSConfig.OFFSET);
        otos.setLinearScalar(DriveConstants.SparkFunOTOSConfig.LINEAR_SCALAR);
        otos.setAngularScalar(DriveConstants.SparkFunOTOSConfig.ANGULAR_SCALAR);
        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(startPose.toSparkFunPose());
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }

    /**
     * Updates the robot's pose estimate based on encoder readings and returns the updated pose.
     *
     * @return the updated {@code Pose2d} of the robot.
     */
    @Override
    public Pose2d update() {
        // since otos auto update, we don't need a #update() method
        return new Pose2d(otos.getPosition());
    }

    /**
     * Sets a new pose estimate for the robot.
     *
     * @param newPose the new {@code Pose2d} to set as the robot's pose estimate.
     */
    @Override
    public void setPoseEstimate(Pose2d newPose) {
        otos.setPosition(newPose.toSparkFunPose());
    }

    /**
     * Gets the current pose estimate of the robot.
     * @return the current {@code Pose2d} estimate of the robot.
     */
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(otos.getPosition());
    }

    /**
     * Gets the velocity of the robot as a {@code Transform2d}.
     *
     * @return the robot's velocity, represented as a {@code Transform2d}.
     */
    @Override
    public Transform2d getVelocity() {
        return new Transform2d(new Pose2d(),new Pose2d(otos.getVelocity()));
    }
}
