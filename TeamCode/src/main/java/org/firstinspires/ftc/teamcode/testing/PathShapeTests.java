package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.CubicPath;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.PathChain;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.paths.PolyLine;

/**
 * Comprehensive test suite for different path shapes and pure pursuit behavior.
 * 
 * <h2>Test Scenarios Included:</h2>
 * <ul>
 * <li><b>Straight Line Test:</b> Basic forward motion validation</li>
 * <li><b>L-Shape Test:</b> Right angle turn handling</li>
 * <li><b>S-Curve Test:</b> Smooth curved path following</li>
 * <li><b>Figure-8 Test:</b> Complex multi-directional curves</li>
 * <li><b>Square Path Test:</b> Sharp corner handling</li>
 * <li><b>Spiral Test:</b> Gradually tightening curve</li>
 * </ul>
 * 
 * <h2>Performance Metrics:</h2>
 * Each test measures:
 * <ul>
 * <li>Completion time</li>
 * <li>Maximum cross-track error</li>
 * <li>Average velocity</li>
 * <li>Path following accuracy</li>
 * </ul>
 * 
 * <h2>Usage Instructions:</h2>
 * <ol>
 * <li>Select this OpMode from the autonomous menu</li>
 * <li>Use gamepad controls to select which test to run</li>
 * <li>Press start to begin the selected test</li>
 * <li>Monitor telemetry for real-time performance data</li>
 * <li>Review performance report at test completion</li>
 * </ol>
 * 
 * @author FTC Team - Path Shape Testing
 * @version 1.0
 */
@Autonomous(name = "Path Shape Tests", group = "Testing")
public class PathShapeTests extends LinearOpMode {
    
    private MecanumDrive drive;
    private int selectedTest = 0;
    private final String[] testNames = {
        "1. Straight Line",
        "2. L-Shape Turn", 
        "3. S-Curve",
        "4. Figure-8",
        "5. Square Path",
        "6. Spiral"
    };
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive system at origin
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        // Test selection phase
        while (!isStarted() && !isStopRequested()) {
            // Use gamepad to select test
            if (gamepad1.dpad_up && System.currentTimeMillis() % 500 < 50) {
                selectedTest = (selectedTest - 1 + testNames.length) % testNames.length;
            }
            if (gamepad1.dpad_down && System.currentTimeMillis() % 500 < 50) {
                selectedTest = (selectedTest + 1) % testNames.length;
            }
            
            telemetry.addData("=== PATH SHAPE TESTS ===", "");
            telemetry.addData("Instructions", "Use DPAD Up/Down to select test");
            telemetry.addData("Selected Test", testNames[selectedTest]);
            telemetry.addData("", "");
            telemetry.addData("Available Tests:", "");
            for (int i = 0; i < testNames.length; i++) {
                String marker = (i == selectedTest) ? ">>> " : "    ";
                telemetry.addData(marker, testNames[i]);
            }
            telemetry.addData("", "Press START to run selected test");
            telemetry.update();
            
            sleep(50);
        }
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Run the selected test
            switch (selectedTest) {
                case 0:
                    runStraightLineTest();
                    break;
                case 1:
                    runLShapeTest();
                    break;
                case 2:
                    runSCurveTest();
                    break;
                case 3:
                    runFigure8Test();
                    break;
                case 4:
                    runSquarePathTest();
                    break;
                case 5:
                    runSpiralTest();
                    break;
            }
            
            // Display final performance report
            displayFinalReport();
        }
    }
    
    /**
     * Test 1: Straight line forward motion.
     * Tests basic pure pursuit functionality and velocity control.
     */
    private void runStraightLineTest() {
        telemetry.addData("Running Test", "Straight Line");
        telemetry.update();
        
        // 4-foot straight line
        LinearPath straightPath = new LinearPath(
            new Point(0, 0),
            new Point(48, 0)
        );
        
        runPathTest(straightPath, "Straight Line Test");
    }
    
    /**
     * Test 2: L-shaped path with a 90-degree turn.
     * Tests pure pursuit handling of sharp direction changes.
     */
    private void runLShapeTest() {
        telemetry.addData("Running Test", "L-Shape Turn");
        telemetry.update();
        
        PolyLine lShapePath = new PolyLine(
            new Point(0, 0),
            new Point(36, 0),    // Go forward 3 feet
            new Point(36, 36)    // Turn left 3 feet
        );
        
        runPathTest(lShapePath, "L-Shape Test");
    }
    
    /**
     * Test 3: Smooth S-curve using cubic Bezier path.
     * Tests pure pursuit with gradual curvature changes.
     */
    private void runSCurveTest() {
        telemetry.addData("Running Test", "S-Curve");
        telemetry.update();
        
        CubicPath sCurvePath = new CubicPath(
            new Point(0, 0),     // Start
            new Point(16, 8),    // Control point 1
            new Point(32, -8),   // Control point 2  
            new Point(48, 0)     // End (same Y as start)
        );
        
        runPathTest(sCurvePath, "S-Curve Test");
    }
    
    /**
     * Test 4: Figure-8 pattern using path chain.
     * Tests complex multi-directional curves and path transitions.
     */
    private void runFigure8Test() {
        telemetry.addData("Running Test", "Figure-8");
        telemetry.update();
        
        // First loop of figure-8
        CubicPath firstLoop = new CubicPath(
            new Point(0, 0),
            new Point(12, 12),
            new Point(24, 12),
            new Point(36, 0)
        );
        
        // Second loop of figure-8 (crossing over)
        CubicPath secondLoop = new CubicPath(
            new Point(36, 0),
            new Point(24, -12),
            new Point(12, -12),
            new Point(0, 0)
        );
        
        PathChain figure8 = new PathChain(firstLoop, secondLoop);
        
        // Use shorter lookahead for tighter curves
        drive.setLookaheadDistance(10.0);
        
        runPathTest(figure8, "Figure-8 Test");
    }
    
    /**
     * Test 5: Square path with sharp 90-degree corners.
     * Tests pure pursuit handling of discontinuous curvature.
     */
    private void runSquarePathTest() {
        telemetry.addData("Running Test", "Square Path");
        telemetry.update();
        
        PolyLine squarePath = new PolyLine(
            new Point(0, 0),
            new Point(24, 0),    // Bottom edge
            new Point(24, 24),   // Right edge
            new Point(0, 24),    // Top edge
            new Point(0, 0)      // Left edge (back to start)
        );
        
        // Use shorter lookahead for sharp corners
        drive.setLookaheadDistance(8.0);
        
        runPathTest(squarePath, "Square Path Test");
    }
    
    /**
     * Test 6: Spiral path with gradually decreasing radius.
     * Tests pure pursuit with continuously changing curvature.
     */
    private void runSpiralTest() {
        telemetry.addData("Running Test", "Spiral");
        telemetry.update();
        
        // Create spiral using multiple cubic segments
        CubicPath spiral1 = new CubicPath(
            new Point(0, 0),
            new Point(24, 0),
            new Point(24, 24),
            new Point(0, 24)
        );
        
        CubicPath spiral2 = new CubicPath(
            new Point(0, 24),
            new Point(-12, 24),
            new Point(-12, 12),
            new Point(0, 12)
        );
        
        CubicPath spiral3 = new CubicPath(
            new Point(0, 12),
            new Point(6, 12),
            new Point(6, 18),
            new Point(0, 18)
        );
        
        PathChain spiralPath = new PathChain(spiral1, spiral2, spiral3);
        
        // Use adaptive lookahead for varying curvature
        drive.setLookaheadDistance(12.0);
        
        runPathTest(spiralPath, "Spiral Test");
    }
    
    /**
     * Generic path test runner with comprehensive metrics collection.
     * 
     * @param path The path to test
     * @param testName Name of the test for telemetry
     */
    private void runPathTest(org.firstinspires.ftc.teamcode.paths.Path path, String testName) {
        long startTime = System.currentTimeMillis();
        
        telemetry.addData("Status", "Starting " + testName);
        telemetry.update();
        
        // Start following the path
        drive.followPath(path);
        
        // Control loop with detailed monitoring
        while (opModeIsActive() && drive.isFollowingPath()) {
            // Allow early exit with B button
            if (gamepad1.b) {
                drive.stopPathFollowing();
                telemetry.addData("Status", "Test stopped by user");
                break;
            }
            
            // Update robot state
            drive.updateLocalizer();
            drive.updatePurePursuit();
            
            // Real-time telemetry (throttled for performance)
            if (System.currentTimeMillis() % 200 < 20) {
                Pose2d currentPose = drive.getLocalizer().getPoseEstimate();
                long elapsedTime = System.currentTimeMillis() - startTime;
                
                telemetry.addData("=== " + testName + " ===", "");
                telemetry.addData("Elapsed Time", elapsedTime / 1000.0 + "s");
                telemetry.addData("Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
                telemetry.addData("Position", String.format("(%.1f, %.1f)", 
                    currentPose.getX(), currentPose.getY()));
                telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));
                telemetry.addData("Cross-Track Error", String.format("%.2f\"", drive.getCrossTrackError()));
                telemetry.addData("Velocity", String.format("%.1f in/s", drive.getCurrentVelocity()));
                telemetry.addData("Lookahead", String.format("%.1f\"", drive.getLookaheadDistance()));
                telemetry.addData("", "Press B to stop test early");
                telemetry.update();
            }
            
            sleep(20); // 50Hz update rate
        }
        
        // Test completion
        long totalTime = System.currentTimeMillis() - startTime;
        telemetry.addData("=== TEST COMPLETE ===", "");
        telemetry.addData("Test", testName);
        telemetry.addData("Total Time", totalTime / 1000.0 + "s");
        telemetry.addData("Final Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
        telemetry.addData("Max Cross-Track Error", String.format("%.2f\"", drive.getMaxCrossTrackError()));
        
        // Performance rating
        if (drive.getMaxCrossTrackError() < 2.0) {
            telemetry.addData("Accuracy Rating", "EXCELLENT");
        } else if (drive.getMaxCrossTrackError() < 4.0) {
            telemetry.addData("Accuracy Rating", "GOOD");
        } else if (drive.getMaxCrossTrackError() < 6.0) {
            telemetry.addData("Accuracy Rating", "FAIR");
        } else {
            telemetry.addData("Accuracy Rating", "NEEDS IMPROVEMENT");
        }
        
        telemetry.update();
        sleep(3000); // Show results for 3 seconds
    }
    
    /**
     * Displays a comprehensive final report with recommendations.
     */
    private void displayFinalReport() {
        String performanceReport = drive.getPerformanceReport();
        String validation = drive.validateTuningParameters();
        
        telemetry.addData("=== FINAL REPORT ===", "");
        telemetry.addLine(performanceReport);
        telemetry.addLine(validation);
        telemetry.addData("", "Test completed. Review data above.");
        telemetry.update();
        
        // Keep display active until stopped
        while (opModeIsActive() && !gamepad1.start) {
            sleep(100);
        }
    }
}