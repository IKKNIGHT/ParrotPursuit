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

    // ================== PURE PURSUIT IMPLEMENTATION ==================

    /**
     * Starts following a path using pure pursuit algorithm
     * @param path the path to follow
     */
    public void followPath(Path path) {
        currentPath = path;
        isFollowingPath = true;
        pathProgress = 0.0;
        lastClosestPoint = null;
        
        // Reset the path to start from the beginning
        path.reset();
        
        telemetry.addData("Pure Pursuit", "Started following path");
    }

    /**
     * Stops path following and stops the robot
     */
    public void stopPathFollowing() {
        isFollowingPath = false;
        currentPath = null;
        setWeightedPowers(0, 0, 0);
        telemetry.addData("Pure Pursuit", "Stopped following path");
    }

    /**
     * Updates the pure pursuit control - call this in your main loop
     */
    public void updatePurePursuit() {
        if (!isFollowingPath || currentPath == null) {
            return;
        }

        Pose2d currentPose = localizer.getPoseEstimate();
        
        if (Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading())) {
            telemetry.addData("Error", "Invalid pose data");
            return;
        }

        // Check if we've completed the path
        if (isPathComplete(currentPose)) {
            stopPathFollowing();
            telemetry.addData("Pure Pursuit", "Path completed!");
            return;
        }

        // Find the closest point on the path
        Point closestPoint = findClosestPointOnPath(currentPose);
        if (closestPoint == null) {
            telemetry.addData("Error", "Could not find closest point on path");
            return;
        }

        // Find the lookahead point
        Point lookaheadPoint = findLookaheadPoint(currentPose, closestPoint);
        if (lookaheadPoint == null) {
            telemetry.addData("Error", "Could not find lookahead point");
            return;
        }

        // Calculate curvature to the lookahead point
        double curvature = calculateCurvature(currentPose, lookaheadPoint);

        // Calculate velocity based on curvature (slow down for sharp turns)
        double targetVelocity = calculateTargetVelocity(curvature);

        // Convert curvature to angular velocity
        double angularVelocity = curvature * targetVelocity;

        // Calculate drive powers
        double forwardPower = targetVelocity / DriveConstants.TunableParams.MAX_VELOCITY;
        double turnPower = angularVelocity / (DriveConstants.TunableParams.MAX_VELOCITY / DriveConstants.TunableParams.TRACK_WIDTH);

        // Limit powers
        forwardPower = Math.max(-1.0, Math.min(1.0, forwardPower));
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        // Apply field-centric control
        driveFieldCentric(forwardPower, 0, turnPower, currentPose.getHeading());

        // Update telemetry
        telemetry.addData("Pure Pursuit", "Following path");
        telemetry.addData("Closest Point", "(" + closestPoint.getX() + ", " + closestPoint.getY() + ")");
        telemetry.addData("Lookahead Point", "(" + lookaheadPoint.getX() + ", " + lookaheadPoint.getY() + ")");
        telemetry.addData("Curvature", curvature);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Forward Power", forwardPower);
        telemetry.addData("Turn Power", turnPower);
    }

    /**
     * Finds the closest point on the current path to the robot's position
     * @param robotPose current robot pose
     * @return closest point on the path
     */
    private Point findClosestPointOnPath(Pose2d robotPose) {
        if (currentPath == null) return null;

        Point robotPoint = new Point(robotPose.getX(), robotPose.getY());
        Point closestPoint = null;
        double minDistance = Double.MAX_VALUE;
        double bestT = 0;

        // Sample the path to find the closest point
        for (double t = 0; t <= 1.0; t += 0.01) {
            Point pathPoint = currentPath.getPoint(t);
            double distance = robotPoint.distance(pathPoint);
            
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = pathPoint;
                bestT = t;
            }
        }

        // Update path progress
        pathProgress = bestT;
        lastClosestPoint = closestPoint;
        
        return closestPoint;
    }

    /**
     * Finds the lookahead point on the path
     * @param robotPose current robot pose
     * @param closestPoint closest point on the path
     * @return lookahead point
     */
    private Point findLookaheadPoint(Pose2d robotPose, Point closestPoint) {
        if (currentPath == null) return null;

        Translation2d robotPos = new Translation2d(robotPose.getX(), robotPose.getY());
        
        // Start searching from the closest point and move forward along the path
        double startT = Math.max(0, pathProgress - 0.1); // Start slightly before closest point
        
        for (double t = startT; t <= 1.0; t += 0.01) {
            Point p1 = currentPath.getPoint(t);
            Point p2 = currentPath.getPoint(Math.min(1.0, t + 0.01));
            
            // Use line-circle intersection to find lookahead point
            Translation2d lineStart = new Translation2d(p1.getX(), p1.getY());
            Translation2d lineEnd = new Translation2d(p2.getX(), p2.getY());
            
            List<Translation2d> intersections = ParrotPursuitUtils.lineCircleIntersection(
                robotPos, lookaheadDistance, lineStart, lineEnd);
            
            if (!intersections.isEmpty()) {
                // Choose the intersection point that's furthest along the path
                Translation2d bestIntersection = intersections.get(0);
                if (intersections.size() > 1) {
                    // Choose the one that's further forward
                    Translation2d intersection1 = intersections.get(0);
                    Translation2d intersection2 = intersections.get(1);
                    
                    if (ParrotPursuitUtils.isInFront(lineStart, lineEnd, intersection2, intersection1)) {
                        bestIntersection = intersection2;
                    }
                }
                
                return new Point(bestIntersection.getX(), bestIntersection.getY());
            }
        }

        // If no intersection found, return the end of the path
        return currentPath.getPoint(1.0);
    }

    /**
     * Calculates the curvature needed to reach the lookahead point
     * @param robotPose current robot pose
     * @param lookaheadPoint target lookahead point
     * @return curvature value
     */
    private double calculateCurvature(Pose2d robotPose, Point lookaheadPoint) {
        // Calculate the angle to the lookahead point
        double dx = lookaheadPoint.getX() - robotPose.getX();
        double dy = lookaheadPoint.getY() - robotPose.getY();
        double absoluteAngleToTarget = Math.atan2(dy, dx);
        
        // Calculate the relative angle (angle in robot's reference frame)
        double relativeAngle = ParrotPursuitUtils.angleWrap(absoluteAngleToTarget - robotPose.getHeading());
        
        // Calculate curvature using the pure pursuit formula
        // curvature = 2 * sin(alpha) / lookahead_distance
        // where alpha is the angle between robot heading and lookahead point
        double curvature = (2.0 * Math.sin(relativeAngle)) / lookaheadDistance;
        
        return curvature;
    }

    /**
     * Calculates target velocity based on path curvature
     * @param curvature current path curvature
     * @return target velocity
     */
    private double calculateTargetVelocity(double curvature) {
        // Slow down for high curvature (sharp turns)
        double curvatureFactor = 1.0 / (1.0 + Math.abs(curvature) * DriveConstants.TunableParams.CURVATURE_VELOCITY_SCALING);
        
        double targetVelocity = DriveConstants.TunableParams.MAX_VELOCITY * curvatureFactor;
        
        // Clamp velocity between min and max
        return Math.max(DriveConstants.TunableParams.MIN_VELOCITY, 
                       Math.min(DriveConstants.TunableParams.MAX_VELOCITY, targetVelocity));
    }

    /**
     * Checks if the path is complete
     * @param robotPose current robot pose
     * @return true if path is complete
     */
    private boolean isPathComplete(Pose2d robotPose) {
        if (currentPath == null) return true;
        
        Point endPoint = currentPath.getPoint(1.0);
        double distanceToEnd = Math.hypot(
            robotPose.getX() - endPoint.getX(),
            robotPose.getY() - endPoint.getY()
        );
        
        return distanceToEnd < DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE;
    }

    /**
     * Updates adaptive lookahead distance based on robot velocity
     */
    private void updateLookaheadDistance() {
        // Adaptive lookahead: faster = longer lookahead
        double velocityFactor = currentVelocity / DriveConstants.TunableParams.MAX_VELOCITY;
        lookaheadDistance = DriveConstants.TunableParams.MIN_LOOKAHEAD_DISTANCE + 
            (DriveConstants.TunableParams.MAX_LOOKAHEAD_DISTANCE - 
             DriveConstants.TunableParams.MIN_LOOKAHEAD_DISTANCE) * velocityFactor;
    }

    /**
     * Gets whether the robot is currently following a path
     * @return true if following a path
     */
    public boolean isFollowingPath() {
        return isFollowingPath;
    }

    /**
     * Gets the current path progress (0.0 to 1.0)
     * @return path progress
     */
    public double getPathProgress() {
        return pathProgress;
    }

    /**
     * Gets the current lookahead distance
     * @return lookahead distance in inches
     */
    public double getLookaheadDistance() {
        return lookaheadDistance;
    }

    /**
     * Sets a custom lookahead distance (overrides adaptive lookahead)
     * @param distance lookahead distance in inches
     */
    public void setLookaheadDistance(double distance) {
        this.lookaheadDistance = Math.max(DriveConstants.TunableParams.MIN_LOOKAHEAD_DISTANCE,
                                         Math.min(DriveConstants.TunableParams.MAX_LOOKAHEAD_DISTANCE, distance));
    }
    
    // ================== LEGACY WAYPOINT METHODS (for backwards compatibility) ==================

    /**
     * Sets motor powers based on weighted feedforward values for forward, strafe, and heading movements.
     * @param front forward movement power.
     * @param strafe strafe (sideways) movement power.
     * @param heading rotational movement power.
     */
    public abstract void setWeightedPowers(double front, double strafe, double heading);
}
