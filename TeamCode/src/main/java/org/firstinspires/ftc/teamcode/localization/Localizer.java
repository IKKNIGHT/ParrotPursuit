/**
 * The {@code Localizer} abstract class serves as a blueprint for implementing localization modules.
 * It defines the necessary methods for updating, setting, and retrieving the robot's pose and velocity.
 */
package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Transform2d;

public abstract class Localizer {

    /**
     * Updates the robot's pose based on sensor readings or calculations.
     *
     * @return the updated {@code Pose2d} of the robot.
     */
    public abstract Pose2d update();

    /**
     * Sets a new pose estimate for the robot.
     *
     * @param newPose the new {@code Pose2d} to set as the robot's pose estimate.
     */
    public abstract void setPoseEstimate(Pose2d newPose);

    /**
     * Retrieves the current pose estimate of the robot.
     *
     * @return the current {@code Pose2d} estimate of the robot.
     */
    public abstract Pose2d getPoseEstimate();

    /**
     * Retrieves the current velocity of the robot.
     *
     * @return the robot's velocity, represented as a {@code Transform2d}.
     */
    public abstract Transform2d getVelocity();
}
