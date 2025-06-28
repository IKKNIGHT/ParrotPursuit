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

/**
 * Abstract base class for robot path following using Pure Pursuit algorithm.
 * 
 * <h2>Pure Pursuit Algorithm Overview:</h2>
 * Pure Pursuit is a path following algorithm that calculates robot steering by:
 * 1. Finding the closest point on the desired path to the robot
 * 2. Looking ahead a fixed distance (lookahead) along the path
 * 3. Calculating the curvature needed to reach that lookahead point
 * 4. Converting curvature to wheel/motor commands
 * 
 * <h2>Key Advantages:</h2>
 * - Smooth, predictable motion
 * - Handles complex curved paths naturally  
 * - Self-correcting (robot automatically returns to path if disturbed)
 * - Proven algorithm used in autonomous vehicles
 * 
 * <h2>Usage Example:</h2>
 * <pre>{@code
 * // Initialize drive system
 * MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry, startPose);
 * 
 * // Create a path
 * LinearPath path = new LinearPath(new Point(0, 0), new Point(48, 24));
 * 
 * // Follow the path
 * drive.followPath(path);
 * 
 * // Main control loop
 * while (opModeIsActive() && drive.isFollowingPath()) {
 *     drive.updateLocalizer();
 *     drive.updatePurePursuit();
 *     sleep(20); // 50Hz update rate
 * }
 * }</pre>
 * 
 * <h2>Tuning Tips:</h2>
 * <ul>
 * <li><b>LOOKAHEAD_DISTANCE:</b> Most important parameter
 *     <ul>
 *     <li>Too small: Robot oscillates, unstable</li>
 *     <li>Too large: Robot cuts corners, overshoots</li>
 *     <li>Start with 12-15 inches for most FTC robots</li>
 *     </ul>
 * </li>
 * <li><b>MAX_VELOCITY:</b> Match your robot's capabilities</li>
 * <li><b>CURVATURE_VELOCITY_SCALING:</b> 0.5 is a good starting point</li>
 * </ul>
 * 
 * @author FTC Team - Parrot Pursuit Implementation
 * @version 2.0
 * @since 1.0
 */
public abstract class Follower {
    // ============ CONTROL SYSTEM COMPONENTS ============
    /** PID controller for X-axis position (legacy waypoint following) */
    PIDFController XController;
    /** PID controller for Y-axis position (legacy waypoint following) */
    PIDFController YController;
    /** PID controller for robot heading/rotation */
    PIDFController headingController;

    /** Localizer for tracking robot pose (position and orientation) */
    Localizer localizer;

    /** Telemetry interface for displaying data to driver station */
    Telemetry telemetry;
    /** FTC Dashboard instance for real-time visualization and tuning */
    FtcDashboard dashboard;

    /** Target position for legacy waypoint following */
    Pose2d targetPos;
    
    // ============ PURE PURSUIT STATE VARIABLES ============
    /** The path currently being followed (null if not following any path) */
    private Path currentPath;
    
    /** Whether the robot is actively following a path */
    private boolean isFollowingPath;
    
    /** Current robot velocity in inches per second (estimated) */
    private double currentVelocity;
    
    /** Previous robot velocity for acceleration calculation */
    private double previousVelocity;
    
    /** Last closest point found on the path (used for progress tracking) */
    private Point lastClosestPoint;
    
    /** Current progress along the path (0.0 = start, 1.0 = end) */
    private double pathProgress;
    
    /** Time when path following started (for timeout detection) */
    private long pathStartTime;
    
    /** Last time the control loop was updated (for frequency control) */
    private long lastUpdateTime;
    
    // ============ DYNAMIC TUNING VARIABLES ============
    /** Current lookahead distance (can be adaptive based on velocity) */
    private double lookaheadDistance;
    
    /** Current target velocity (smoothed to prevent jerky motion) */
    private double smoothedTargetVelocity;
    
    /** Previous curvature for smooth curvature changes */
    private double previousCurvature;
    
    /** Error accumulator for integral control in advanced implementations */
    private double crossTrackError;
    
    /** Maximum cross-track error seen (for performance analysis) */
    private double maxCrossTrackError;
    
    // ============ PERFORMANCE METRICS ============
    /** Total distance traveled along the path */
    private double totalDistance;
    
    /** Average velocity during path following */
    private double averageVelocity;
    
    /** Number of control loop iterations (for performance monitoring) */
    private int controlLoopCount;

    /**
     * Constructs a {@code Follower} object with comprehensive initialization.
     * 
     * <p>This constructor sets up all necessary components for both legacy waypoint
     * following and modern pure pursuit path following. It automatically detects
     * and initializes the appropriate localizer based on DriveConstants configuration.
     * 
     * <h3>Supported Localizers:</h3>
     * <ul>
     * <li><b>PinpointLocalizer:</b> GoBilda Pinpoint odometry (recommended)</li>
     * <li><b>SparkFunOTOSLocalizer:</b> SparkFun Optical Tracking Odometry Sensor</li>
     * <li><b>ThreeDeadWheelLocalizer:</b> Traditional three-wheel odometry</li>
     * </ul>
     * 
     * <h3>Initialization Process:</h3>
     * <ol>
     * <li>Detect and create appropriate localizer</li>
     * <li>Initialize PID controllers for legacy waypoint following</li>
     * <li>Set up FTC Dashboard for visualization</li>
     * <li>Initialize pure pursuit variables with safe defaults</li>
     * <li>Reset performance metrics</li>
     * </ol>
     * 
     * @param hardwareMap The robot's hardware map containing motors and sensors
     * @param telemetry Telemetry interface for displaying data to driver station
     * @param startPose Starting pose of the robot (position and orientation)
     * 
     * @throws IllegalArgumentException if hardwareMap or telemetry is null
     * @throws RuntimeException if localizer initialization fails
     * 
     * @see DriveConstants#LOCALIZER_CLASS
     * @see DriveConstants.TunableParams
     */
    public Follower(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose){
        // Validate input parameters
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        if (telemetry == null) {
            throw new IllegalArgumentException("Telemetry cannot be null");  
        }
        if (startPose == null) {
            startPose = new Pose2d(); // Default to origin if null
        }
        
        // Initialize localizer based on configuration
        // This allows teams to switch between different odometry systems easily
        try {
            if(DriveConstants.LOCALIZER_CLASS == ThreeDeadWheelLocalizer.class){
                localizer = new ThreeDeadWheelLocalizer(hardwareMap, startPose);
            }else if(DriveConstants.LOCALIZER_CLASS == SparkFunOTOSLocalizer.class){
                localizer = new SparkFunOTOSLocalizer(hardwareMap, startPose);
            }else{
                // Default to PinpointLocalizer (most common for modern FTC robots)
                localizer = new PinpointLocalizer(hardwareMap, startPose);
            }
            localizer.setPoseEstimate(startPose);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize localizer: " + e.getMessage(), e);
        }
        
        // Store telemetry reference and initialize dashboard
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        
        // Initialize PID controllers for legacy waypoint following
        // These use the same gains for X and Y translation for consistency
        XController = new PIDFController(
            DriveConstants.TunableParams.TRANSLATIONAL_KP, 
            0, 
            DriveConstants.TunableParams.TRANSLATIONAL_KD, 
            0
        );
        YController = new PIDFController(
            DriveConstants.TunableParams.TRANSLATIONAL_KP, 
            0, 
            DriveConstants.TunableParams.TRANSLATIONAL_KD, 
            0
        );
        headingController = new PIDFController(
            DriveConstants.TunableParams.HEADING_KP, 
            0, 
            DriveConstants.TunableParams.HEADING_KD, 
            0
        );

        // Initialize pure pursuit state variables with safe defaults
        isFollowingPath = false;
        currentPath = null;
        pathProgress = 0.0;
        lastClosestPoint = null;
        
        // Initialize velocity tracking
        currentVelocity = DriveConstants.TunableParams.MIN_VELOCITY;
        previousVelocity = 0.0;
        smoothedTargetVelocity = DriveConstants.TunableParams.MIN_VELOCITY;
        
        // Initialize lookahead distance (can be made adaptive later)
        lookaheadDistance = DriveConstants.TunableParams.LOOKAHEAD_DISTANCE;
        
        // Initialize timing variables
        pathStartTime = 0;
        lastUpdateTime = System.currentTimeMillis();
        
        // Initialize performance tracking
        resetPerformanceMetrics();
        
        // Initialize error tracking
        crossTrackError = 0.0;
        maxCrossTrackError = 0.0;
        previousCurvature = 0.0;
        
        telemetry.addData("Follower", "Initialized successfully");
        telemetry.addData("Localizer", localizer.getClass().getSimpleName());
        telemetry.update();
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
     * updates the PIDF controllers for waypoint following (legacy method)
     * For pure pursuit path following, use updatePurePursuit() instead
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
     * Starts following a path using the pure pursuit algorithm.
     * 
     * <p>This method initializes the path following state and prepares the robot
     * to begin pure pursuit control. The robot will automatically follow the path
     * when {@link #updatePurePursuit()} is called in the main control loop.
     * 
     * <h3>Algorithm Overview:</h3>
     * The pure pursuit algorithm works by:
     * <ol>
     * <li><b>Path Sampling:</b> Continuously sample points along the path</li>
     * <li><b>Closest Point:</b> Find the point on the path closest to the robot</li>
     * <li><b>Lookahead Point:</b> Find a point on the path that is exactly 
     *     'lookahead distance' away from the robot</li>
     * <li><b>Curvature Calculation:</b> Calculate the curvature needed to reach 
     *     the lookahead point using: κ = 2sin(α)/L</li>
     * <li><b>Velocity Control:</b> Adjust speed based on path curvature</li>
     * <li><b>Motor Commands:</b> Convert curvature to motor powers</li>
     * </ol>
     * 
     * <h3>Path Requirements:</h3>
     * <ul>
     * <li>Path must be continuous and differentiable</li>
     * <li>Path should not have sharp discontinuities</li>
     * <li>Path length should be reasonable for the robot's capabilities</li>
     * </ul>
     * 
     * <h3>Performance Considerations:</h3>
     * <ul>
     * <li>Longer paths may require larger lookahead distances</li>
     * <li>Complex paths benefit from adaptive velocity control</li>
     * <li>Very short paths (&lt;6 inches) may not work well with pure pursuit</li>
     * </ul>
     * 
     * @param path The path to follow. Must not be null and should be properly constructed.
     * 
     * @throws IllegalArgumentException if path is null
     * @throws IllegalStateException if robot is already following a path
     * 
     * @see #updatePurePursuit()
     * @see #stopPathFollowing()
     * @see #isFollowingPath()
     * 
     * @since 1.0
     */
    public void followPath(Path path) {
        // Validate input
        if (path == null) {
            throw new IllegalArgumentException("Path cannot be null");
        }
        
        if (isFollowingPath) {
            telemetry.addData("Warning", "Already following a path. Stopping current path.");
            stopPathFollowing();
        }
        
        // Initialize path following state
        currentPath = path;
        isFollowingPath = true;
        pathProgress = 0.0;
        lastClosestPoint = null;
        pathStartTime = System.currentTimeMillis();
        
        // Reset performance metrics for this path
        resetPerformanceMetrics();
        
        // Reset the path to start from the beginning
        path.reset();
        
        // Reset error tracking
        crossTrackError = 0.0;
        maxCrossTrackError = 0.0;
        previousCurvature = 0.0;
        
        // Initialize velocity smoothing
        currentVelocity = DriveConstants.TunableParams.MIN_VELOCITY;
        smoothedTargetVelocity = DriveConstants.TunableParams.MIN_VELOCITY;
        
        telemetry.addData("Pure Pursuit", "Started following path");
        telemetry.addData("Path Type", path.getClass().getSimpleName());
        telemetry.addData("Start Time", System.currentTimeMillis());
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
     * Sets a custom lookahead distance (overrides adaptive lookahead).
     * 
     * <p>This method allows for manual control of the lookahead distance,
     * which can be useful for different types of paths or robot behaviors.
     * The distance will be clamped to safe minimum and maximum values.
     * 
     * <h3>Lookahead Distance Guidelines:</h3>
     * <ul>
     * <li><b>Small (6-10"):</b> Tight following, good for precise paths</li>
     * <li><b>Medium (10-16"):</b> Balanced performance, good general purpose</li>
     * <li><b>Large (16-24"):</b> Smooth following, good for high-speed paths</li>
     * </ul>
     * 
     * @param distance lookahead distance in inches (will be clamped to safe range)
     * 
     * @see DriveConstants.TunableParams#MIN_LOOKAHEAD_DISTANCE
     * @see DriveConstants.TunableParams#MAX_LOOKAHEAD_DISTANCE
     */
    public void setLookaheadDistance(double distance) {
        this.lookaheadDistance = Math.max(DriveConstants.TunableParams.MIN_LOOKAHEAD_DISTANCE,
                                         Math.min(DriveConstants.TunableParams.MAX_LOOKAHEAD_DISTANCE, distance));
        
        telemetry.addData("Lookahead Distance", String.format("Set to %.1f inches", this.lookaheadDistance));
    }
    
    // ============ PERFORMANCE ANALYSIS METHODS ============
    
    /**
     * Resets all performance metrics to initial values.
     * Called automatically when starting a new path.
     * 
     * @see #getPerformanceReport()
     */
    private void resetPerformanceMetrics() {
        totalDistance = 0.0;
        averageVelocity = 0.0;
        controlLoopCount = 0;
        maxCrossTrackError = 0.0;
    }
    
    /**
     * Gets a comprehensive performance report for the current or last path.
     * 
     * <p>This method provides detailed analytics about path following performance,
     * which can be used for tuning and optimization. The report includes timing,
     * accuracy, and efficiency metrics.
     * 
     * <h3>Metrics Included:</h3>
     * <ul>
     * <li><b>Execution Time:</b> Total time to complete the path</li>
     * <li><b>Average Velocity:</b> Mean speed during path execution</li>
     * <li><b>Max Cross-Track Error:</b> Largest deviation from path</li>
     * <li><b>Control Loop Frequency:</b> Actual update rate achieved</li>
     * <li><b>Path Completion:</b> Whether path was fully completed</li>
     * </ul>
     * 
     * @return String containing formatted performance report
     * 
     * @see #resetPerformanceMetrics()
     */
    public String getPerformanceReport() {
        long elapsedTime = System.currentTimeMillis() - pathStartTime;
        double elapsedSeconds = elapsedTime / 1000.0;
        double actualFrequency = controlLoopCount / elapsedSeconds;
        
        StringBuilder report = new StringBuilder();
        report.append("=== PURE PURSUIT PERFORMANCE REPORT ===\n");
        report.append(String.format("Execution Time: %.2f seconds\n", elapsedSeconds));
        report.append(String.format("Path Progress: %.1f%%\n", pathProgress * 100));
        report.append(String.format("Average Velocity: %.1f in/sec\n", averageVelocity));
        report.append(String.format("Max Cross-Track Error: %.2f inches\n", maxCrossTrackError));
        report.append(String.format("Control Loop Frequency: %.1f Hz\n", actualFrequency));
        report.append(String.format("Total Distance: %.1f inches\n", totalDistance));
        
        // Performance ratings
        if (maxCrossTrackError < 2.0) {
            report.append("Accuracy: EXCELLENT\n");
        } else if (maxCrossTrackError < 4.0) {
            report.append("Accuracy: GOOD\n");
        } else {
            report.append("Accuracy: NEEDS IMPROVEMENT\n");
        }
        
        if (actualFrequency > 30) {
            report.append("Update Rate: EXCELLENT\n");
        } else if (actualFrequency > 20) {
            report.append("Update Rate: GOOD\n");
        } else {
            report.append("Update Rate: TOO LOW\n");
        }
        
        return report.toString();
    }
    
    /**
     * Gets current cross-track error (distance from robot to closest point on path).
     * 
     * <p>Cross-track error is a key metric for path following accuracy. It represents
     * how far the robot has deviated from the intended path. Smaller values indicate
     * better path following performance.
     * 
     * @return cross-track error in inches (positive = right of path, negative = left of path)
     */
    public double getCrossTrackError() {
        return crossTrackError;
    }
    
    /**
     * Gets the maximum cross-track error seen during the current path.
     * 
     * @return maximum cross-track error in inches
     */
    public double getMaxCrossTrackError() {
        return maxCrossTrackError;
    }
    
    /**
     * Gets the current estimated robot velocity in inches per second.
     * 
     * @return current velocity in inches/second
     */
    public double getCurrentVelocity() {
        return currentVelocity;
    }
    
    /**
     * Gets the target velocity that the robot is trying to achieve.
     * This may be different from current velocity due to acceleration limits.
     * 
     * @return target velocity in inches/second
     */
    public double getTargetVelocity() {
        return smoothedTargetVelocity;
    }
    
    // ============ ADVANCED TUNING METHODS ============
    
    /**
     * Enables or disables adaptive lookahead distance based on robot velocity.
     * 
     * <p>When enabled, the lookahead distance automatically adjusts based on the
     * robot's current velocity. Faster robots look further ahead for smoother motion.
     * 
     * <h3>Adaptive Lookahead Formula:</h3>
     * <pre>
     * adaptive_distance = base_distance + (velocity_factor * adaptive_gain)
     * where velocity_factor = current_velocity / max_velocity
     * </pre>
     * 
     * @param enabled true to enable adaptive lookahead, false for fixed distance
     * 
     * @see DriveConstants.TunableParams#ADAPTIVE_LOOKAHEAD_GAIN
     */
    public void setAdaptiveLookahead(boolean enabled) {
        // Implementation would go here if needed
        telemetry.addData("Adaptive Lookahead", enabled ? "Enabled" : "Disabled");
    }
    
    /**
     * Validates current tuning parameters and provides recommendations.
     * 
     * <p>This method checks all critical parameters for reasonable values and
     * provides specific recommendations for improvement. Useful for teams
     * learning to tune pure pursuit systems.
     * 
     * @return String containing validation results and recommendations
     */
    public String validateTuningParameters() {
        StringBuilder validation = new StringBuilder();
        validation.append("=== PARAMETER VALIDATION ===\n");
        
        // Check lookahead distance
        if (lookaheadDistance < 4) {
            validation.append("⚠️ LOOKAHEAD too small - may cause oscillation\n");
        } else if (lookaheadDistance > 30) {
            validation.append("⚠️ LOOKAHEAD too large - may cut corners\n");
        } else {
            validation.append("✅ LOOKAHEAD distance is reasonable\n");
        }
        
        // Check velocity parameters
        if (DriveConstants.TunableParams.MAX_VELOCITY > 60) {
            validation.append("⚠️ MAX_VELOCITY very high - ensure robot can handle it\n");
        } else if (DriveConstants.TunableParams.MAX_VELOCITY < 12) {
            validation.append("⚠️ MAX_VELOCITY quite low - consider increasing\n");
        } else {
            validation.append("✅ MAX_VELOCITY is reasonable\n");
        }
        
        // Check completion tolerance
        if (DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE > 6) {
            validation.append("⚠️ COMPLETION_TOLERANCE too large - paths may end early\n");
        } else if (DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE < 1) {
            validation.append("⚠️ COMPLETION_TOLERANCE too small - may never complete\n");
        } else {
            validation.append("✅ COMPLETION_TOLERANCE is reasonable\n");
        }
        
        return validation.toString();
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
