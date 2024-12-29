/**
 * The {@code PinpointLocalizer} class is a localization module that uses a {@code GoBildaPinpointDriver}
 * to determine the robot's pose and velocity. This localizer provides real-time updates on the
 * robot's position and orientation and allows for setting and retrieving pose estimates.
 */
package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Transform2d;

public class PinpointLocalizer extends Localizer {

    private GoBildaPinpointDriver odo;

    /**
     * Constructs a {@code PinpointLocalizer} with a default starting pose of (0, 0, 0).
     *
     * @param hwMap the hardware map used to retrieve the {@code GoBildaPinpointDriver}.
     */
    public PinpointLocalizer(HardwareMap hwMap){
        this(hwMap, new Pose2d());
    }

    /**
     * Constructs a {@code PinpointLocalizer} with a specified starting pose.
     *
     * @param hwMap the hardware map used to retrieve the {@code GoBildaPinpointDriver}.
     * @param startPose the initial pose estimate for the robot.
     */
    public PinpointLocalizer(HardwareMap hwMap, Pose2d startPose){
        odo = hwMap.get(GoBildaPinpointDriver.class, DriveConstants.PinpointConfig.PINPOINT_NAME);
        odo.setOffsets(DriveConstants.PinpointConfig.PINPOINT_X_OFFSET, DriveConstants.PinpointConfig.PINPOINT_Y_OFFSET);
        odo.setEncoderResolution(DriveConstants.PinpointConfig.POD); // change this to odo.setEncoderResolution(double resolution); if using custom odometry pods
        odo.setEncoderDirections(DriveConstants.PinpointConfig.X_DIRECTION, DriveConstants.PinpointConfig.Y_DIRECTION);

        odo.setPosition(startPose.toPinpointPose());
    }

    /**
     * Updates the robot's pose estimate based on encoder readings and returns the updated pose.
     *
     * @return the updated {@code Pose2d} of the robot.
     */
    @Override
    public Pose2d update() {
        odo.update();
        return new Pose2d(odo.getPosition());
    }

    /**
     * Sets a new pose estimate for the robot.
     *
     * @param newPose the new {@code Pose2d} to set as the robot's pose estimate.
     */
    @Override
    public void setPoseEstimate(Pose2d newPose) {
        odo.setPosition(newPose.toPinpointPose());
    }

    /**
     * Gets the current pose estimate of the robot.
     *
     * @return the current {@code Pose2d} estimate of the robot.
     */
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(odo.getPosition());
    }

    /**
     * Gets the velocity of the robot as a {@code Transform2d}.
     *
     * @return the robot's velocity, represented as a {@code Transform2d}.
     */
    @Override
    public Transform2d getVelocity() {
        return new Transform2d(new Pose2d(), new Pose2d(odo.getVelocity()));
    }
}
