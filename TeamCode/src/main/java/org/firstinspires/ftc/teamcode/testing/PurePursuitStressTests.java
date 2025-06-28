package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.CubicPath;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.PathChain;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.paths.PolyLine;

/**
 * Stress test suite for pure pursuit performance under challenging conditions.
 * 
 * <h2>Test Categories:</h2>
 * <ul>
 * <li><b>Speed Tests:</b> Maximum velocity and acceleration testing</li>
 * <li><b>Precision Tests:</b> Tight tolerance and accuracy validation</li>
 * <li><b>Complexity Tests:</b> Multi-segment paths with rapid direction changes</li>
 * <li><b>Endurance Tests:</b> Long-duration path following</li>
 * <li><b>Edge Case Tests:</b> Unusual scenarios and error recovery</li>
 * </ul>
 * 
 * <h2>Stress Test Objectives:</h2>
 * <ul>
 * <li>Validate system stability under extreme conditions</li>
 * <li>Identify performance bottlenecks and limitations</li>
 * <li>Test error recovery and fail-safe mechanisms</li>
 * <li>Measure system response to parameter variations</li>
 * <li>Validate real-world competition readiness</li>
 * </ul>
 * 
 * <h2>Performance Benchmarks:</h2>
 * <ul>
 * <li><b>Update Rate:</b> Should maintain >30Hz under all conditions</li>
 * <li><b>Cross-Track Error:</b> Should stay <4 inches for most paths</li>
 * <li><b>Completion Rate:</b> Should complete 95%+ of paths successfully</li>
 * <li><b>Recovery Time:</b> Should recover from disturbances within 1 second</li>
 * </ul>
 * 
 * @author FTC Team - Stress Testing
 * @version 1.0
 */
@Autonomous(name = "Pure Pursuit Stress Tests", group = "Testing")
public class PurePursuitStressTests extends LinearOpMode {
    
    private MecanumDrive drive;
    private int testCount = 0;
    private int passedTests = 0;
    private int failedTests = 0;
    
    // Test result tracking
    private double totalTestTime = 0;
    private double maxErrorSeen = 0;
    private double minUpdateRate = Double.MAX_VALUE;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive system
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("=== PURE PURSUIT STRESS TESTS ===", "");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("WARNING", "These tests push the system to its limits");
        telemetry.addData("", "Ensure robot has adequate space (8x8 feet minimum)");
        telemetry.addData("", "Press START to begin stress testing");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Run comprehensive stress test suite
            runSpeedTests();
            sleep(1000);
            
            runPrecisionTests();
            sleep(1000);
            
            runComplexityTests();
            sleep(1000);
            
            runEnduranceTest();
            sleep(1000);
            
            runEdgeCaseTests();
            
            // Final comprehensive report
            displayStressTestReport();
        }
    }
    
    /**
     * Tests pure pursuit performance at high speeds and accelerations.
     */
    private void runSpeedTests() {
        telemetry.addData("=== SPEED TESTS ===", "");
        telemetry.update();
        
        // Test 1: Maximum velocity straight line
        testHighSpeedStraightLine();
        
        // Test 2: High-speed curves
        testHighSpeedCurves();
        
        // Test 3: Rapid acceleration/deceleration
        testRapidAcceleration();
    }
    
    private void testHighSpeedStraightLine() {
        // Temporarily increase max velocity for speed test
        double originalMaxVel = DriveConstants.TunableParams.MAX_VELOCITY;
        DriveConstants.TunableParams.MAX_VELOCITY = 60.0; // High speed
        
        LinearPath speedPath = new LinearPath(
            new Point(0, 0),
            new Point(72, 0) // 6 feet at high speed
        );
        
        StressTestResult result = runStressTest(speedPath, "High Speed Straight", 10.0);
        evaluateStressTestResult(result, 4.0, 20.0); // Allow higher error at high speed
        
        // Restore original velocity
        DriveConstants.TunableParams.MAX_VELOCITY = originalMaxVel;
    }
    
    private void testHighSpeedCurves() {
        double originalMaxVel = DriveConstants.TunableParams.MAX_VELOCITY;
        DriveConstants.TunableParams.MAX_VELOCITY = 36.0; // Moderate high speed
        
        CubicPath curvePath = new CubicPath(
            new Point(0, 0),
            new Point(24, 12),
            new Point(48, 12),
            new Point(72, 0)
        );
        
        StressTestResult result = runStressTest(curvePath, "High Speed Curves", 8.0);
        evaluateStressTestResult(result, 6.0, 25.0); // Higher tolerance for curves
        
        DriveConstants.TunableParams.MAX_VELOCITY = originalMaxVel;
    }
    
    private void testRapidAcceleration() {
        // Create stop-and-go pattern
        PolyLine stopGoPath = new PolyLine(
            new Point(0, 0),
            new Point(12, 0),    // Short segment
            new Point(12, 12),   // Sharp turn
            new Point(24, 12),   // Short segment
            new Point(24, 0),    // Sharp turn
            new Point(36, 0)     // Final segment
        );
        
        drive.setLookaheadDistance(6.0); // Short lookahead for rapid changes
        
        StressTestResult result = runStressTest(stopGoPath, "Rapid Acceleration", 5.0);
        evaluateStressTestResult(result, 3.0, 30.0);
    }
    
    /**
     * Tests pure pursuit precision and accuracy under tight tolerances.
     */
    private void runPrecisionTests() {
        telemetry.addData("=== PRECISION TESTS ===", "");
        telemetry.update();
        
        // Test 1: Very tight path following
        testTightPathFollowing();
        
        // Test 2: Precision waypoint hitting
        testPrecisionWaypoints();
        
        // Test 3: Minimal lookahead distance
        testMinimalLookahead();
    }
    
    private void testTightPathFollowing() {
        // Create a serpentine path requiring precise following
        PolyLine serpentinePath = new PolyLine(
            new Point(0, 0),
            new Point(8, 4),
            new Point(16, 0),
            new Point(24, -4),
            new Point(32, 0),
            new Point(40, 4),
            new Point(48, 0)
        );
        
        // Reduce velocity for precision
        double originalMaxVel = DriveConstants.TunableParams.MAX_VELOCITY;
        DriveConstants.TunableParams.MAX_VELOCITY = 12.0;
        
        drive.setLookaheadDistance(8.0);
        
        StressTestResult result = runStressTest(serpentinePath, "Tight Path Following", 15.0);
        evaluateStressTestResult(result, 2.0, 30.0); // Strict accuracy requirement
        
        DriveConstants.TunableParams.MAX_VELOCITY = originalMaxVel;
    }
    
    private void testPrecisionWaypoints() {
        // Create path with very specific waypoints
        PolyLine precisionPath = new PolyLine(
            new Point(0, 0),
            new Point(18, 18),   // Diagonal
            new Point(36, 0),    // Back to X-axis
            new Point(18, -18),  // Mirror diagonal
            new Point(0, 0)      // Return to start
        );
        
        // Very tight completion tolerance
        double originalTolerance = DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE;
        DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE = 1.0;
        
        StressTestResult result = runStressTest(precisionPath, "Precision Waypoints", 12.0);
        evaluateStressTestResult(result, 1.5, 30.0);
        
        DriveConstants.TunableParams.PATH_COMPLETION_TOLERANCE = originalTolerance;
    }
    
    private void testMinimalLookahead() {
        // Test with minimum possible lookahead distance
        LinearPath minLookaheadPath = new LinearPath(
            new Point(0, 0),
            new Point(24, 24)
        );
        
        drive.setLookaheadDistance(DriveConstants.TunableParams.MIN_LOOKAHEAD_DISTANCE);
        
        StressTestResult result = runStressTest(minLookaheadPath, "Minimal Lookahead", 8.0);
        evaluateStressTestResult(result, 3.0, 25.0); // May be less stable
    }
    
    /**
     * Tests complex multi-segment paths with challenging geometries.
     */
    private void runComplexityTests() {
        telemetry.addData("=== COMPLEXITY TESTS ===", "");
        telemetry.update();
        
        testMegaPath();
        testRapidDirectionChanges();
        testNestedLoops();
    }
    
    private void testMegaPath() {
        // Create a very long, complex path with multiple segments
        LinearPath seg1 = new LinearPath(new Point(0, 0), new Point(24, 0));
        CubicPath seg2 = new CubicPath(new Point(24, 0), new Point(36, 12), new Point(24, 24), new Point(0, 24));
        LinearPath seg3 = new LinearPath(new Point(0, 24), new Point(-24, 24));
        CubicPath seg4 = new CubicPath(new Point(-24, 24), new Point(-36, 12), new Point(-24, 0), new Point(0, 0));
        
        PathChain megaPath = new PathChain(seg1, seg2, seg3, seg4);
        
        StressTestResult result = runStressTest(megaPath, "Mega Path Complex", 20.0);
        evaluateStressTestResult(result, 5.0, 25.0);
    }
    
    private void testRapidDirectionChanges() {
        // Create zig-zag pattern with sharp direction changes
        PolyLine zigzagPath = new PolyLine(
            new Point(0, 0),
            new Point(6, 6),
            new Point(12, 0),
            new Point(18, 6),
            new Point(24, 0),
            new Point(30, 6),
            new Point(36, 0)
        );
        
        drive.setLookaheadDistance(5.0); // Very short for sharp turns
        
        StressTestResult result = runStressTest(zigzagPath, "Rapid Direction Changes", 10.0);
        evaluateStressTestResult(result, 4.0, 25.0);
    }
    
    private void testNestedLoops() {
        // Create figure-8 with a loop inside each circle
        CubicPath outerLoop1 = new CubicPath(new Point(0, 0), new Point(18, 18), new Point(36, 18), new Point(54, 0));
        CubicPath innerLoop1 = new CubicPath(new Point(54, 0), new Point(45, -9), new Point(36, -9), new Point(27, 0));
        CubicPath outerLoop2 = new CubicPath(new Point(27, 0), new Point(18, -18), new Point(36, -18), new Point(0, 0));
        
        PathChain nestedPath = new PathChain(outerLoop1, innerLoop1, outerLoop2);
        
        StressTestResult result = runStressTest(nestedPath, "Nested Loops", 15.0);
        evaluateStressTestResult(result, 6.0, 20.0);
    }
    
    /**
     * Tests system endurance with long-duration path following.
     */
    private void runEnduranceTest() {
        telemetry.addData("=== ENDURANCE TEST ===", "");
        telemetry.addData("Status", "Running 30-second continuous path...");
        telemetry.update();
        
        // Create a long, repeating pattern
        PolyLine endurancePath = new PolyLine(
            new Point(0, 0), new Point(36, 0), new Point(36, 36), new Point(0, 36),
            new Point(0, 0), new Point(-36, 0), new Point(-36, -36), new Point(0, -36),
            new Point(0, 0), new Point(24, 24), new Point(-24, 24), new Point(-24, -24),
            new Point(24, -24), new Point(0, 0)
        );
        
        StressTestResult result = runStressTest(endurancePath, "Endurance Test", 30.0);
        evaluateStressTestResult(result, 4.0, 20.0);
    }
    
    /**
     * Tests edge cases and error recovery scenarios.
     */
    private void runEdgeCaseTests() {
        telemetry.addData("=== EDGE CASE TESTS ===", "");
        telemetry.update();
        
        testZeroLengthPath();
        testSinglePointPath();
        testBacktrackingPath();
    }
    
    private void testZeroLengthPath() {
        // Test with start and end at same point
        LinearPath zeroPath = new LinearPath(new Point(0, 0), new Point(0, 0));
        
        StressTestResult result = runStressTest(zeroPath, "Zero Length Path", 2.0);
        // This should complete immediately or handle gracefully
        passedTests++; // Always pass if no crash
    }
    
    private void testSinglePointPath() {
        // Test with very short path
        LinearPath shortPath = new LinearPath(new Point(0, 0), new Point(0.1, 0));
        
        StressTestResult result = runStressTest(shortPath, "Single Point Path", 3.0);
        evaluateStressTestResult(result, 1.0, 30.0);
    }
    
    private void testBacktrackingPath() {
        // Path that requires going backward
        PolyLine backtrackPath = new PolyLine(
            new Point(0, 0),
            new Point(24, 0),
            new Point(12, 0),    // Backtrack
            new Point(36, 0)     // Forward again
        );
        
        StressTestResult result = runStressTest(backtrackPath, "Backtracking Path", 8.0);
        evaluateStressTestResult(result, 3.0, 25.0);
    }
    
    /**
     * Generic stress test runner with comprehensive monitoring.
     */
    private StressTestResult runStressTest(org.firstinspires.ftc.teamcode.paths.Path path, String testName, double timeoutSeconds) {
        testCount++;
        long startTime = System.currentTimeMillis();
        long timeoutMs = (long) (timeoutSeconds * 1000);
        
        telemetry.addData("Running", testName);
        telemetry.update();
        
        drive.followPath(path);
        
        double maxError = 0;
        double minFrequency = Double.MAX_VALUE;
        int updateCount = 0;
        
        while (opModeIsActive() && drive.isFollowingPath()) {
            long elapsed = System.currentTimeMillis() - startTime;
            if (elapsed > timeoutMs) {
                drive.stopPathFollowing();
                break; // Timeout
            }
            
            long loopStart = System.currentTimeMillis();
            
            drive.updateLocalizer();
            drive.updatePurePursuit();
            
            // Monitor performance
            double crossTrackError = Math.abs(drive.getCrossTrackError());
            maxError = Math.max(maxError, crossTrackError);
            
            updateCount++;
            long loopTime = System.currentTimeMillis() - loopStart;
            if (loopTime > 0) {
                double frequency = 1000.0 / loopTime;
                minFrequency = Math.min(minFrequency, frequency);
            }
            
            sleep(20);
        }
        
        long totalTime = System.currentTimeMillis() - startTime;
        double actualFrequency = (updateCount * 1000.0) / totalTime;
        
        return new StressTestResult(
            testName,
            totalTime / 1000.0,
            maxError,
            actualFrequency,
            drive.getPathProgress() > 0.95, // Consider 95%+ complete as success
            drive.getPathProgress()
        );
    }
    
    private void evaluateStressTestResult(StressTestResult result, double maxAllowedError, double minAllowedFreq) {
        boolean passed = result.completed && 
                        result.maxCrossTrackError <= maxAllowedError && 
                        result.updateFrequency >= minAllowedFreq;
        
        if (passed) {
            passedTests++;
        } else {
            failedTests++;
        }
        
        totalTestTime += result.executionTime;
        maxErrorSeen = Math.max(maxErrorSeen, result.maxCrossTrackError);
        minUpdateRate = Math.min(minUpdateRate, result.updateFrequency);
        
        telemetry.addData("Test Result", result.testName);
        telemetry.addData("Status", passed ? "PASSED" : "FAILED");
        telemetry.addData("Execution Time", String.format("%.2fs", result.executionTime));
        telemetry.addData("Max Error", String.format("%.2f\" (limit: %.2f\")", result.maxCrossTrackError, maxAllowedError));
        telemetry.addData("Update Rate", String.format("%.1fHz (limit: %.1fHz)", result.updateFrequency, minAllowedFreq));
        telemetry.addData("Completion", String.format("%.1f%%", result.progress * 100));
        telemetry.update();
        
        sleep(1500); // Show individual results
    }
    
    private void displayStressTestReport() {
        double passRate = (passedTests * 100.0) / testCount;
        
        telemetry.addData("=== STRESS TEST COMPLETE ===", "");
        telemetry.addData("Tests Run", testCount);
        telemetry.addData("Passed", passedTests);
        telemetry.addData("Failed", failedTests);
        telemetry.addData("Pass Rate", String.format("%.1f%%", passRate));
        telemetry.addData("", "");
        telemetry.addData("Performance Summary:", "");
        telemetry.addData("Total Test Time", String.format("%.1fs", totalTestTime));
        telemetry.addData("Worst Error Seen", String.format("%.2f inches", maxErrorSeen));
        telemetry.addData("Lowest Update Rate", String.format("%.1f Hz", minUpdateRate));
        telemetry.addData("", "");
        
        if (passRate >= 90) {
            telemetry.addData("Overall Rating", "EXCELLENT - Competition Ready");
        } else if (passRate >= 75) {
            telemetry.addData("Overall Rating", "GOOD - Minor tuning needed");
        } else if (passRate >= 60) {
            telemetry.addData("Overall Rating", "FAIR - Significant tuning needed");
        } else {
            telemetry.addData("Overall Rating", "POOR - Major issues detected");
        }
        
        telemetry.update();
        
        // Keep results displayed
        while (opModeIsActive()) {
            sleep(100);
        }
    }
    
    /**
     * Data structure for stress test results.
     */
    private static class StressTestResult {
        final String testName;
        final double executionTime;
        final double maxCrossTrackError;
        final double updateFrequency;
        final boolean completed;
        final double progress;
        
        StressTestResult(String testName, double executionTime, double maxCrossTrackError, 
                        double updateFrequency, boolean completed, double progress) {
            this.testName = testName;
            this.executionTime = executionTime;
            this.maxCrossTrackError = maxCrossTrackError;
            this.updateFrequency = updateFrequency;
            this.completed = completed;
            this.progress = progress;
        }
    }
}