package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.controllers.PIDFController;

public abstract class Follower {
    PIDFController XController;
    PIDFController YController;
    PIDFController headingController;

    // Localizer for tracking robot pose
    Localizer localizer;

    // Current pose of the robot
    public Pose2d pose;

    public Follower(HardwareMap hardwareMap, Pose2d startPose){
        this.pose = startPose;
        // Initialize localizer (uses PinpointLocalizer by default)
        if (DriveConstants.LOCALIZER_CLASS == PinpointLocalizer.class) {
            localizer = new PinpointLocalizer(hardwareMap);
        }
    }

    public abstract void setWeightedPowers(double front, double strafe, double heading);
}
