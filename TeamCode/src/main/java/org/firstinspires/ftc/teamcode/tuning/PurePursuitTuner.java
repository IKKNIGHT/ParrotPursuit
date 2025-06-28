package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.Point;

/**
 * OpMode for tuning Pure Pursuit parameters using FTC Dashboard
 * Connect to FTC Dashboard to adjust parameters in real-time
 */
@Config
@Autonomous(name = "Pure Pursuit Tuner", group = "Tuning")
public class PurePursuitTuner extends LinearOpMode {
    
    // Tunable parameters via FTC Dashboard
    public static double LOOKAHEAD_DISTANCE = 12.0;
    public static double MIN_LOOKAHEAD = 6.0;
    public static double MAX_LOOKAHEAD = 24.0;
    public static double PATH_COMPLETION_TOLERANCE = 2.0;
    public static double MAX_VELOCITY = 24.0;
    public static double MIN_VELOCITY = 6.0;
    public static double CURVATURE_VELOCITY_SCALING = 0.5;
    
    // Test path parameters
    public static double PATH_LENGTH = 48.0;
    public static double PATH_WIDTH = 24.0;
    public static boolean USE_SQUARE_PATH = false;
    
    private MecanumDrive drive;
    private FtcDashboard dashboard;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        
        // Initialize the drive system
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Use FTC Dashboard to adjust parameters");
        telemetry.addData("Dashboard URL", "192.168.43.1:8080/dash");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update constants from dashboard
            updateConstants();
            
            // Create test path based on parameters
            LinearPath testPath;
            if (USE_SQUARE_PATH) {
                // Create a more complex path for advanced tuning
                testPath = new LinearPath(
                    new Point(0, 0),
                    new Point(PATH_LENGTH, PATH_WIDTH)
                );
            } else {
                // Simple straight line for basic tuning
                testPath = new LinearPath(
                    new Point(0, 0),
                    new Point(PATH_LENGTH, 0)
                );
            }
            
            telemetry.addData("Status", "Ready to test");
            telemetry.addData("Instructions", "Press A to run test path");
            telemetry.addData("Current Settings", "");
            telemetry.addData("  Lookahead Distance", LOOKAHEAD_DISTANCE);
            telemetry.addData("  Max Velocity", MAX_VELOCITY);
            telemetry.addData("  Path Tolerance", PATH_COMPLETION_TOLERANCE);
            telemetry.update();
            
            // Wait for user input
            while (opModeIsActive() && !gamepad1.a) {
                updateConstants();
                sleep(50);
            }
            
            if (!opModeIsActive()) break;
            
            // Run the test
            runTuningTest(testPath);
            
            // Wait for button release
            while (opModeIsActive() && gamepad1.a) {
                sleep(50);
            }
        }
    }
    
    private void runTuningTest(LinearPath testPath) {
        telemetry.addData("Status", "Running test...");
        telemetry.update();
        
        // Set custom lookahead distance
        drive.setLookaheadDistance(LOOKAHEAD_DISTANCE);
        
        // Start following the path
        drive.followPath(testPath);
        
        long startTime = System.currentTimeMillis();
        
        // Control loop
        while (opModeIsActive() && drive.isFollowingPath()) {
            // Allow early exit with B button
            if (gamepad1.b) {
                drive.stopPathFollowing();
                break;
            }
            
            // Update constants in real-time
            updateConstants();
            drive.setLookaheadDistance(LOOKAHEAD_DISTANCE);
            
            // Update robot
            drive.updateLocalizer();
            drive.updatePurePursuit();
            
            // Detailed telemetry for tuning
            Pose2d currentPose = drive.getLocalizer().getPoseEstimate();
            long elapsedTime = System.currentTimeMillis() - startTime;
            
            telemetry.addData("=== TEST STATUS ===", "");
            telemetry.addData("Elapsed Time", elapsedTime / 1000.0 + "s");
            telemetry.addData("Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
            telemetry.addData("Position", String.format("(%.1f, %.1f)", currentPose.getX(), currentPose.getY()));
            telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()) + "°");
            
            telemetry.addData("=== CURRENT SETTINGS ===", "");
            telemetry.addData("Lookahead Distance", LOOKAHEAD_DISTANCE);
            telemetry.addData("Max Velocity", MAX_VELOCITY);
            telemetry.addData("Min Velocity", MIN_VELOCITY);
            telemetry.addData("Curvature Scaling", CURVATURE_VELOCITY_SCALING);
            telemetry.addData("Path Tolerance", PATH_COMPLETION_TOLERANCE);
            
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("B Button", "Stop test early");
            
            telemetry.update();
            
            sleep(20); // 50Hz
        }
        
        long totalTime = System.currentTimeMillis() - startTime;
        Pose2d finalPose = drive.getLocalizer().getPoseEstimate();
        
        telemetry.addData("=== TEST COMPLETE ===", "");
        telemetry.addData("Total Time", totalTime / 1000.0 + "s");
        telemetry.addData("Final Position", String.format("(%.1f, %.1f)", finalPose.getX(), finalPose.getY()));
        telemetry.addData("Target was", String.format("(%.1f, %.1f)", PATH_LENGTH, USE_SQUARE_PATH ? PATH_WIDTH : 0.0));
        
        // Calculate accuracy
        double targetX = PATH_LENGTH;
        double targetY = USE_SQUARE_PATH ? PATH_WIDTH : 0.0;
        double error = Math.hypot(finalPose.getX() - targetX, finalPose.getY() - targetY);
        telemetry.addData("Position Error", String.format("%.1f inches", error));
        
        if (error < 3.0) {
            telemetry.addData("Result", "EXCELLENT (< 3 inches)");
        } else if (error < 6.0) {
            telemetry.addData("Result", "GOOD (< 6 inches)");
        } else if (error < 12.0) {
            telemetry.addData("Result", "FAIR (< 12 inches)");
        } else {
            telemetry.addData("Result", "NEEDS TUNING (> 12 inches)");
        }
        
        telemetry.update();
        
        sleep(3000); // Show results for 3 seconds
    }
    
    private void updateConstants() {
        // Update DriveConstants with dashboard values
        DriveConstants.TunableParams.LOOKAHEAD_DISTANCE = LOOKAHEAD_DISTANCE;
        DriveConstants.TunableParams.MIN_LOOKAHEAD_DISTANCE = MIN_LOOKAHEAD;
        DriveConstants.TunableParams.MAX_LOOKAHEAD_DISTANCE = MAX_LOOKAHEAD;
        DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE = PATH_COMPLETION_TOLERANCE;
        DriveConstants.TunableParams.MAX_VELOCITY = MAX_VELOCITY;
        DriveConstants.TunableParams.MIN_VELOCITY = MIN_VELOCITY;
        DriveConstants.TunableParams.CURVATURE_VELOCITY_SCALING = CURVATURE_VELOCITY_SCALING;
    }
}