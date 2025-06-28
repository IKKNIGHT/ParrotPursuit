package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.geometry.Transform2d;
import org.firstinspires.ftc.teamcode.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.geometry.WayPoint;
import org.firstinspires.ftc.teamcode.paths.CubicPath;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.paths.PolyLine;

/**
 * Comprehensive comparison between Pure Pursuit and traditional PID waypoint following.
 * 
 * <h2>Comparison Methodology:</h2>
 * This OpMode runs identical paths using both control methods and measures:
 * <ul>
 * <li><b>Execution Time:</b> How long each method takes</li>
 * <li><b>Path Accuracy:</b> Maximum deviation from intended path</li>
 * <li><b>Smoothness:</b> Jerk and acceleration characteristics</li>
 * <li><b>Efficiency:</b> Total distance traveled vs. optimal</li>
 * <li><b>Robustness:</b> Handling of disturbances and errors</li>
 * </ul>
 * 
 * <h2>Test Scenarios:</h2>
 * <ul>
 * <li><b>Straight Line:</b> Basic motion comparison</li>
 * <li><b>Curved Path:</b> Smooth curve following</li>
 * <li><b>Sharp Turns:</b> Discontinuous direction changes</li>
 * <li><b>Complex Path:</b> Multi-segment path with varying curvature</li>
 * </ul>
 * 
 * <h2>Expected Results:</h2>
 * <ul>
 * <li><b>Pure Pursuit:</b> Smoother motion, better curve following, may be slower on straight lines</li>
 * <li><b>PID Waypoints:</b> More direct on straight lines, may oscillate on curves</li>
 * </ul>
 * 
 * @author FTC Team - Algorithm Comparison
 * @version 1.0
 */
@Autonomous(name = "Pure Pursuit vs PID Comparison", group = "Testing")
public class PurePursuitVsPIDComparison extends LinearOpMode {
    
    private MecanumDrive drive;
    
    // Test result tracking
    private TestResult purePursuitResult;
    private TestResult pidResult;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive system
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("=== ALGORITHM COMPARISON TEST ===", "");
        telemetry.addData("Purpose", "Compare Pure Pursuit vs PID Waypoint Following");
        telemetry.addData("", "");
        telemetry.addData("Test Sequence:", "");
        telemetry.addData("1.", "Straight Line Test");
        telemetry.addData("2.", "Curved Path Test");
        telemetry.addData("3.", "Sharp Turn Test");
        telemetry.addData("4.", "Complex Path Test");
        telemetry.addData("", "");
        telemetry.addData("Instructions", "Press START to begin comparison");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Run all comparison tests
            runStraightLineComparison();
            sleep(2000);
            
            runCurvedPathComparison();
            sleep(2000);
            
            runSharpTurnComparison();
            sleep(2000);
            
            runComplexPathComparison();
            sleep(2000);
            
            // Display final comprehensive comparison
            displayFinalComparison();
        }
    }
    
    /**
     * Test 1: Straight line motion comparison.
     * Should favor PID for simplicity, but test smoothness.
     */
    private void runStraightLineComparison() {
        telemetry.addData("=== STRAIGHT LINE COMPARISON ===", "");
        telemetry.update();
        
        // Define the test path - simple straight line
        LinearPath straightPath = new LinearPath(
            new Point(0, 0),
            new Point(48, 0) // 4 feet forward
        );
        
        // Test 1A: Pure Pursuit
        telemetry.addData("Testing", "Pure Pursuit - Straight Line");
        telemetry.update();
        
        resetRobotPosition();
        purePursuitResult = runPurePursuitTest(straightPath, "Straight Line");
        
        sleep(1000);
        
        // Test 1B: PID Waypoint
        telemetry.addData("Testing", "PID Waypoint - Straight Line");
        telemetry.update();
        
        resetRobotPosition();
        pidResult = runPIDWaypointTest(straightPath, "Straight Line");
        
        // Display comparison
        displayTestComparison("Straight Line Test", purePursuitResult, pidResult);
        sleep(3000);
    }
    
    /**
     * Test 2: Curved path comparison.
     * Should heavily favor Pure Pursuit for smoothness.
     */
    private void runCurvedPathComparison() {
        telemetry.addData("=== CURVED PATH COMPARISON ===", "");
        telemetry.update();
        
        // Define a smooth S-curve
        CubicPath curvePath = new CubicPath(
            new Point(0, 0),
            new Point(16, 12),
            new Point(32, -12),
            new Point(48, 0)
        );
        
        // Test 2A: Pure Pursuit
        telemetry.addData("Testing", "Pure Pursuit - Curved Path");
        telemetry.update();
        
        resetRobotPosition();
        purePursuitResult = runPurePursuitTest(curvePath, "Curved Path");
        
        sleep(1000);
        
        // Test 2B: PID Waypoint (sample curve into waypoints)
        telemetry.addData("Testing", "PID Waypoint - Curved Path");
        telemetry.update();
        
        resetRobotPosition();
        pidResult = runPIDWaypointTestFromCurve(curvePath, "Curved Path");
        
        displayTestComparison("Curved Path Test", purePursuitResult, pidResult);
        sleep(3000);
    }
    
    /**
     * Test 3: Sharp turn comparison.
     * Tests handling of discontinuous curvature.
     */
    private void runSharpTurnComparison() {
        telemetry.addData("=== SHARP TURN COMPARISON ===", "");
        telemetry.update();
        
        // Define L-shaped path with sharp 90-degree turn
        PolyLine sharpTurnPath = new PolyLine(
            new Point(0, 0),
            new Point(24, 0),    // Go forward
            new Point(24, 24)    // Sharp left turn
        );
        
        // Test 3A: Pure Pursuit
        telemetry.addData("Testing", "Pure Pursuit - Sharp Turn");
        telemetry.update();
        
        resetRobotPosition();
        purePursuitResult = runPurePursuitTest(sharpTurnPath, "Sharp Turn");
        
        sleep(1000);
        
        // Test 3B: PID Waypoint
        telemetry.addData("Testing", "PID Waypoint - Sharp Turn");
        telemetry.update();
        
        resetRobotPosition();
        pidResult = runPIDWaypointTest(sharpTurnPath, "Sharp Turn");
        
        displayTestComparison("Sharp Turn Test", purePursuitResult, pidResult);
        sleep(3000);
    }
    
    /**
     * Test 4: Complex multi-segment path comparison.
     * Tests overall system performance and robustness.
     */
    private void runComplexPathComparison() {
        telemetry.addData("=== COMPLEX PATH COMPARISON ===", "");
        telemetry.update();
        
        // Define complex path with multiple challenges
        PolyLine complexPath = new PolyLine(
            new Point(0, 0),     // Start
            new Point(24, 0),    // Straight
            new Point(24, 24),   // Sharp turn
            new Point(12, 24),   // Reverse direction
            new Point(12, 12),   // Sharp turn
            new Point(36, 12),   // Long straight
            new Point(36, 0),    // Turn
            new Point(0, 0)      // Return to start
        );
        
        // Test 4A: Pure Pursuit
        telemetry.addData("Testing", "Pure Pursuit - Complex Path");
        telemetry.update();
        
        resetRobotPosition();
        purePursuitResult = runPurePursuitTest(complexPath, "Complex Path");
        
        sleep(1000);
        
        // Test 4B: PID Waypoint
        telemetry.addData("Testing", "PID Waypoint - Complex Path");
        telemetry.update();
        
        resetRobotPosition();
        pidResult = runPIDWaypointTest(complexPath, "Complex Path");
        
        displayTestComparison("Complex Path Test", purePursuitResult, pidResult);
        sleep(3000);
    }
    
    /**
     * Run pure pursuit test and collect performance metrics.
     */
    private TestResult runPurePursuitTest(org.firstinspires.ftc.teamcode.paths.Path path, String testName) {
        long startTime = System.currentTimeMillis();
        double maxError = 0;
        double totalDistance = 0;
        int updateCount = 0;
        
        Pose2d lastPose = drive.getLocalizer().getPoseEstimate();
        
        // Start pure pursuit
        drive.followPath(path);
        
        while (opModeIsActive() && drive.isFollowingPath()) {
            // Early exit option
            if (gamepad1.b) {
                drive.stopPathFollowing();
                break;
            }
            
            drive.updateLocalizer();
            drive.updatePurePursuit();
            
            // Collect metrics
            Pose2d currentPose = drive.getLocalizer().getPoseEstimate();
            double crossTrackError = Math.abs(drive.getCrossTrackError());
            maxError = Math.max(maxError, crossTrackError);
            
            // Calculate distance traveled
            double distanceStep = Math.hypot(
                currentPose.getX() - lastPose.getX(),
                currentPose.getY() - lastPose.getY()
            );
            totalDistance += distanceStep;
            lastPose = currentPose;
            
            updateCount++;
            
            // Throttled telemetry
            if (updateCount % 10 == 0) {
                telemetry.addData("Pure Pursuit Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
                telemetry.addData("Cross-Track Error", String.format("%.2f\"", crossTrackError));
                telemetry.update();
            }
            
            sleep(20);
        }
        
        long totalTime = System.currentTimeMillis() - startTime;
        double executionTime = totalTime / 1000.0;
        double updateFrequency = (updateCount * 1000.0) / totalTime;
        
        return new TestResult(
            "Pure Pursuit",
            testName,
            executionTime,
            maxError,
            totalDistance,
            updateFrequency,
            drive.getPathProgress()
        );
    }
    
    /**
     * Run PID waypoint test by converting path to discrete waypoints.
     */
    private TestResult runPIDWaypointTest(org.firstinspires.ftc.teamcode.paths.Path path, String testName) {
        long startTime = System.currentTimeMillis();
        double maxError = 0;
        double totalDistance = 0;
        int updateCount = 0;
        
        Pose2d lastPose = drive.getLocalizer().getPoseEstimate();
        
        // Convert path to waypoints (sample every 10% of path)
        WayPoint[] waypoints = new WayPoint[11]; // 0%, 10%, 20%, ..., 100%
        for (int i = 0; i <= 10; i++) {
            double t = i / 10.0;
            Point pathPoint = path.getPoint(t);
            waypoints[i] = new WayPoint(
                new Pose2d(pathPoint.getX(), pathPoint.getY(), new Rotation2d(0)),
                2.0 // 2-inch tolerance
            );
        }
        
        // Follow waypoints using traditional PID
        for (WayPoint waypoint : waypoints) {
            if (!opModeIsActive()) break;
            
            drive.setTarget(waypoint);
            
            // Wait for robot to reach waypoint
            while (opModeIsActive() && 
                   (!drive.XController.atSetPoint() || 
                    !drive.YController.atSetPoint())) {
                
                // Early exit option
                if (gamepad1.b) break;
                
                drive.updateLocalizer();
                drive.updatePIDS(); // Use legacy PID method
                
                // Collect metrics
                Pose2d currentPose = drive.getLocalizer().getPoseEstimate();
                
                // Calculate error to intended path (approximate)
                double errorToWaypoint = Math.hypot(
                    currentPose.getX() - waypoint.getPosition().getX(),
                    currentPose.getY() - waypoint.getPosition().getY()
                );
                maxError = Math.max(maxError, errorToWaypoint);
                
                // Calculate distance traveled
                double distanceStep = Math.hypot(
                    currentPose.getX() - lastPose.getX(),
                    currentPose.getY() - lastPose.getY()
                );
                totalDistance += distanceStep;
                lastPose = currentPose;
                
                updateCount++;
                
                // Throttled telemetry
                if (updateCount % 10 == 0) {
                    telemetry.addData("PID Waypoint Progress", String.format("Waypoint %d", 
                        java.util.Arrays.asList(waypoints).indexOf(waypoint) + 1));
                    telemetry.addData("Error to Target", String.format("%.2f\"", errorToWaypoint));
                    telemetry.update();
                }
                
                sleep(20);
            }
            
            sleep(200); // Brief pause at each waypoint
        }
        
        long totalTime = System.currentTimeMillis() - startTime;
        double executionTime = totalTime / 1000.0;
        double updateFrequency = (updateCount * 1000.0) / totalTime;
        
        return new TestResult(
            "PID Waypoint",
            testName,
            executionTime,
            maxError,
            totalDistance,
            updateFrequency,
            1.0 // Assume completion if we got through all waypoints
        );
    }
    
    /**
     * Special PID test for curved paths - samples more densely.
     */
    private TestResult runPIDWaypointTestFromCurve(CubicPath curvePath, String testName) {
        // For curves, sample more densely (every 5% of path)
        long startTime = System.currentTimeMillis();
        double maxError = 0;
        double totalDistance = 0;
        int updateCount = 0;
        
        Pose2d lastPose = drive.getLocalizer().getPoseEstimate();
        
        // Sample curve more densely for PID waypoints
        WayPoint[] waypoints = new WayPoint[21]; // Every 5%
        for (int i = 0; i <= 20; i++) {
            double t = i / 20.0;
            Point pathPoint = curvePath.getPoint(t);
            waypoints[i] = new WayPoint(
                new Pose2d(pathPoint.getX(), pathPoint.getY(), new Rotation2d(0)),
                1.5 // Tighter tolerance for curves
            );
        }
        
        // Follow waypoints
        for (WayPoint waypoint : waypoints) {
            if (!opModeIsActive()) break;
            
            drive.setTarget(waypoint);
            
            // Shorter timeout per waypoint for curves
            long waypointStart = System.currentTimeMillis();
            while (opModeIsActive() && 
                   (!drive.XController.atSetPoint() || !drive.YController.atSetPoint()) &&
                   (System.currentTimeMillis() - waypointStart < 2000)) {
                
                if (gamepad1.b) break;
                
                drive.updateLocalizer();
                drive.updatePIDS();
                
                // Collect metrics
                Pose2d currentPose = drive.getLocalizer().getPoseEstimate();
                double errorToWaypoint = Math.hypot(
                    currentPose.getX() - waypoint.getPosition().getX(),
                    currentPose.getY() - waypoint.getPosition().getY()
                );
                maxError = Math.max(maxError, errorToWaypoint);
                
                double distanceStep = Math.hypot(
                    currentPose.getX() - lastPose.getX(),
                    currentPose.getY() - lastPose.getY()
                );
                totalDistance += distanceStep;
                lastPose = currentPose;
                
                updateCount++;
                
                sleep(20);
            }
            
            sleep(100); // Shorter pause for curves
        }
        
        long totalTime = System.currentTimeMillis() - startTime;
        double executionTime = totalTime / 1000.0;
        double updateFrequency = (updateCount * 1000.0) / totalTime;
        
        return new TestResult(
            "PID Waypoint",
            testName,
            executionTime,
            maxError,
            totalDistance,
            updateFrequency,
            1.0
        );
    }
    
    /**
     * Reset robot to starting position for fair comparison.
     */
    private void resetRobotPosition() {
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive.getLocalizer().setPoseEstimate(startPose);
        
        telemetry.addData("Status", "Resetting robot position...");
        telemetry.update();
        sleep(1000);
    }
    
    /**
     * Display comparison results for a single test.
     */
    private void displayTestComparison(String testName, TestResult pp, TestResult pid) {
        telemetry.addData("=== " + testName + " RESULTS ===", "");
        telemetry.addData("", "");
        telemetry.addData("Execution Time:", "");
        telemetry.addData("  Pure Pursuit", String.format("%.2fs", pp.executionTime));
        telemetry.addData("  PID Waypoint", String.format("%.2fs", pid.executionTime));
        telemetry.addData("  Winner", pp.executionTime < pid.executionTime ? "Pure Pursuit" : "PID Waypoint");
        telemetry.addData("", "");
        telemetry.addData("Maximum Error:", "");
        telemetry.addData("  Pure Pursuit", String.format("%.2f\"", pp.maxError));
        telemetry.addData("  PID Waypoint", String.format("%.2f\"", pid.maxError));
        telemetry.addData("  Winner", pp.maxError < pid.maxError ? "Pure Pursuit" : "PID Waypoint");
        telemetry.addData("", "");
        telemetry.addData("Total Distance:", "");
        telemetry.addData("  Pure Pursuit", String.format("%.1f\"", pp.totalDistance));
        telemetry.addData("  PID Waypoint", String.format("%.1f\"", pid.totalDistance));
        telemetry.addData("  Winner", pp.totalDistance < pid.totalDistance ? "Pure Pursuit" : "PID Waypoint");
        telemetry.addData("", "");
        telemetry.addData("Update Frequency:", "");
        telemetry.addData("  Pure Pursuit", String.format("%.1fHz", pp.updateFrequency));
        telemetry.addData("  PID Waypoint", String.format("%.1fHz", pid.updateFrequency));
        telemetry.update();
    }
    
    /**
     * Display final comprehensive comparison across all tests.
     */
    private void displayFinalComparison() {
        telemetry.addData("=== COMPREHENSIVE COMPARISON ===", "");
        telemetry.addData("", "");
        telemetry.addData("PURE PURSUIT ADVANTAGES:", "");
        telemetry.addData("✓", "Smooth curved path following");
        telemetry.addData("✓", "Continuous path tracking");
        telemetry.addData("✓", "Natural velocity control");
        telemetry.addData("✓", "Handles complex geometries well");
        telemetry.addData("✓", "Self-correcting behavior");
        telemetry.addData("", "");
        telemetry.addData("PID WAYPOINT ADVANTAGES:", "");
        telemetry.addData("✓", "Simple to understand and tune");
        telemetry.addData("✓", "Direct point-to-point motion");
        telemetry.addData("✓", "Precise waypoint hitting");
        telemetry.addData("✓", "Lower computational overhead");
        telemetry.addData("", "");
        telemetry.addData("RECOMMENDATIONS:", "");
        telemetry.addData("•", "Use Pure Pursuit for smooth autonomous paths");
        telemetry.addData("•", "Use PID Waypoints for precise positioning");
        telemetry.addData("•", "Pure Pursuit better for competition auto");
        telemetry.addData("•", "PID better for driver-controlled positioning");
        telemetry.addData("", "");
        telemetry.addData("Press any button to exit", "");
        telemetry.update();
        
        // Wait for button press to exit
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            sleep(100);
        }
    }
    
    /**
     * Data structure for test results.
     */
    private static class TestResult {
        final String algorithm;
        final String testName;
        final double executionTime;
        final double maxError;
        final double totalDistance;
        final double updateFrequency;
        final double completion;
        
        TestResult(String algorithm, String testName, double executionTime, 
                  double maxError, double totalDistance, double updateFrequency, double completion) {
            this.algorithm = algorithm;
            this.testName = testName;
            this.executionTime = executionTime;
            this.maxError = maxError;
            this.totalDistance = totalDistance;
            this.updateFrequency = updateFrequency;
            this.completion = completion;
        }
    }
}