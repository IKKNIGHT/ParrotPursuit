package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.geometry.WayPoint;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.localization.SparkFunOTOSLocalizer;
import org.firstinspires.ftc.teamcode.localization.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.utils.ParrotPursuitUtils;
import org.firstinspires.ftc.teamcode.utils.controllers.PIDFController;

import java.util.List;

public abstract class Follower {
    PIDFController XController;
    PIDFController YController;
    PIDFController headingController;

    // Localizer for tracking robot pose
    Localizer localizer;

    Telemetry telemetry;
    FtcDashboard dashboard;

    Pose2d targetPos;
    
    // Pure Pursuit Path Following Variables
    private Path currentPath;
    private boolean isFollowingPath;
    private double currentVelocity;
    private Point lastClosestPoint;
    private double pathProgress;
    
    // Lookahead distance (can be adaptive)
    private double lookaheadDistance;

    /**
     * Constructs a {@code Follower} object.
     * @param hardwareMap hardwareMap with your odo/motors
     * @param telemetry telemetry for logging
     * @param startPose starting pose of the robot
     */
    public Follower(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose){
        // Initialize localizer (uses PinpointLocalizer by default)
        if(DriveConstants.LOCALIZER_CLASS == ThreeDeadWheelLocalizer.class){
            localizer = new ThreeDeadWheelLocalizer(hardwareMap, startPose);
        }else if(DriveConstants.LOCALIZER_CLASS == SparkFunOTOSLocalizer.class){
            localizer = new SparkFunOTOSLocalizer(hardwareMap, startPose);
        }else{
            localizer = new PinpointLocalizer(hardwareMap, startPose);
        }
        localizer.setPoseEstimate(startPose);
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        
        // Initialize PID controllers
        XController = new PIDFController(DriveConstants.TunableParams.TRANSLATIONAL_KP, 0, DriveConstants.TunableParams.TRANSLATIONAL_KD, 0);
        YController = new PIDFController(DriveConstants.TunableParams.TRANSLATIONAL_KP, 0, DriveConstants.TunableParams.TRANSLATIONAL_KD, 0);
        headingController = new PIDFController(DriveConstants.TunableParams.HEADING_KP, 0, DriveConstants.TunableParams.HEADING_KD, 0);

        // Initialize pure pursuit variables
        isFollowingPath = false;
        currentVelocity = DriveConstants.TunableParams.MIN_VELOCITY;
        lookaheadDistance = DriveConstants.TunableParams.LOOKAHEAD_DISTANCE;
        pathProgress = 0.0;
    }

    /**
     * sets the target position
     * @param target target waypoint
     */
    public void setTarget(WayPoint target){
        XController.setSetPoint(target.getPosition().getX());
        YController.setSetPoint(target.getPosition().getY());
        headingController.setSetPoint(target.getPosition().getHeading());

        XController.setTolerance(target.getTolerance().getTranslation().getX());
        YController.setTolerance(target.getTolerance().getTranslation().getY());
        headingController.setTolerance(target.getTolerance().getRotation().getRadians());
    }

    /**
     * updates the localizer position and on dashboard
     */
    public void updateLocalizer(){
        Pose2d pos = localizer.update();
        telemetry.addData("position", pos.getX()+" "+pos.getY()+" "+pos.getHeading());
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setFill("blue")
                .strokeCircle(pos.getX(), pos.getY(), 5)
                .strokeLine(pos.getX(), pos.getY(),
                        (Math.cos(pos.getHeading())*5)+ pos.getX(),
                        (Math.sin(pos.getHeading())*5)+ pos.getY());

        // Add path visualization if following a path
        if (isFollowingPath && currentPath != null) {
            // Draw the current path
            for (double t = 0; t <= 1.0; t += 0.05) {
                Point pathPoint = currentPath.getPoint(t);
                packet.fieldOverlay().setFill("red").strokeCircle(pathPoint.getX(), pathPoint.getY(), 2);
            }
            
            // Draw lookahead circle
            packet.fieldOverlay().setStroke("green")
                    .strokeCircle(pos.getX(), pos.getY(), lookaheadDistance);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * updates the PIDF controllers
     */
    public void updatePIDS(){
        Pose2d currPos = localizer.getPoseEstimate();
        if (Double.isNaN(currPos.getX()) || Double.isNaN(currPos.getY()) || Double.isNaN(currPos.getHeading())) {
            System.out.println("THE INPUT IS NAN");
            return;
        }
        double XPower = XController.calculate(currPos.getX());
        double YPower = YController.calculate(currPos.getY());
        double headingPower = headingController.calculate(currPos.getHeading());
        driveFieldCentric(XPower, YPower, headingPower, currPos.getHeading());
    }
    /**
     * gets the target position
     * @return target position
     */
    public Pose2d getTargetPos(){
        return new Pose2d(XController.getSetPoint(), YController.getSetPoint(),new Rotation2d(headingController.getSetPoint()));
    }
    @Deprecated
    public Localizer getLocalizer(){return localizer;}

    /**
     * drives the robot using field centric coordinates
     * @param XPower How much to move forward
     * @param YPower How much to move sideways
     * @param turnPower How much to turn
     * @param currHeading The current heading of the robot
     */
    public void driveFieldCentric(double XPower, double YPower, double turnPower, double currHeading){
        double x = XPower * Math.cos(currHeading) + YPower * Math.sin(currHeading);
        double y = YPower * Math.cos(currHeading) - XPower * Math.sin(currHeading);
        setWeightedPowers(x, y, turnPower);
    }

    /**
     * Sets motor powers based on weighted feedforward values for forward, strafe, and heading movements.
     * @param front forward movement power.
     * @param strafe strafe (sideways) movement power.
     * @param heading rotational movement power.
     */
    public abstract void setWeightedPowers(double front, double strafe, double heading);
}
