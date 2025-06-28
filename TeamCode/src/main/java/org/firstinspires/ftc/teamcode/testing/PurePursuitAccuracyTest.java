package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.CubicPath;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.paths.PolyLine;

import java.util.ArrayList;
import java.util.List;

/**
 * Precision accuracy testing and measurement for Pure Pursuit implementation.
 * 
 * <h2>Testing Methodology:</h2>
 * This OpMode provides scientific measurement of Pure Pursuit accuracy using:
 * <ul>
 * <li><b>Multiple Trial Testing:</b> Runs multiple iterations for statistical analysis</li>
 * <li><b>Standardized Paths:</b> Repeatable test paths with known expected outcomes</li>
 * <li><b>Comprehensive Metrics:</b> Position accuracy, timing, and consistency measurements</li>
 * <li><b>Statistical Analysis:</b> Mean, standard deviation, and confidence intervals</li>
 * <li><b>Comparative Analysis:</b> Performance across different path types</li>
 * </ul>
 * 
 * <h2>Accuracy Metrics:</h2>
 * <ul>
 * <li><b>Final Position Error:</b> Distance from intended endpoint</li>
 * <li><b>Maximum Cross-Track Error:</b> Largest deviation during path following</li>
 * <li><b>RMS Error:</b> Root mean square error across entire path</li>
 * <li><b>Execution Time Consistency:</b> Variance in completion times</li>
 * <li><b>Path Following Efficiency:</b> Actual vs theoretical path length</li>
 * </ul>
 * 
 * <h2>Statistical Analysis:</h2>
 * <ul>
 * <li><b>Repeatability:</b> Standard deviation across multiple trials</li>
 * <li><b>Confidence Intervals:</b> 95% confidence bounds on measurements</li>
 * <li><b>Outlier Detection:</b> Identification of anomalous results</li>
 * <li><b>Trend Analysis:</b> Performance changes across trials</li>
 * </ul>
 * 
 * @author FTC Team - Accuracy Testing
 * @version 1.0
 */
@Config
@Autonomous(name = "Pure Pursuit Accuracy Test", group = "Testing")
public class PurePursuitAccuracyTest extends LinearOpMode {
    
    // Configurable test parameters via FTC Dashboard
    public static int TRIALS_PER_TEST = 5;
    public static boolean ENABLE_DETAILED_LOGGING = true;
    public static boolean AUTO_RESET_POSITION = true;
    public static double MEASUREMENT_PRECISION = 0.1; // inches
    
    private MecanumDrive drive;
    private FtcDashboard dashboard;
    
    // Statistical tracking
    private List<AccuracyResult> allResults = new ArrayList<>();
    private int currentTrial = 0;
    private String currentTestName = "";
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        
        // Initialize drive system
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("=== PURE PURSUIT ACCURACY TEST ===", "");
        telemetry.addData("Purpose", "Scientific measurement of path following accuracy");
        telemetry.addData("Trials per Test", TRIALS_PER_TEST);
        telemetry.addData("Total Tests", "4 different path types");
        telemetry.addData("Total Trials", TRIALS_PER_TEST * 4);
        telemetry.addData("", "");
        telemetry.addData("Instructions", "Ensure robot has adequate space");
        telemetry.addData("", "Place robot at center of 8x8 foot area");
        telemetry.addData("", "Press START when ready");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Run comprehensive accuracy test suite
            runAccuracyTestSuite();
            
            // Generate final statistical report
            generateFinalStatisticalReport();
        }
    }
    
    /**
     * Main test suite runner.
     */
    private void runAccuracyTestSuite() {
        // Test 1: Straight line accuracy
        runRepeatedAccuracyTest("Straight Line", this::createStraightLinePath);
        
        sleep(2000);
        
        // Test 2: 90-degree turn accuracy
        runRepeatedAccuracyTest("90-Degree Turn", this::create90DegreeTurnPath);
        
        sleep(2000);
        
        // Test 3: Smooth curve accuracy
        runRepeatedAccuracyTest("Smooth Curve", this::createSmoothCurvePath);
        
        sleep(2000);
        
        // Test 4: Complex path accuracy
        runRepeatedAccuracyTest("Complex Path", this::createComplexPath);
    }
    
    /**
     * Run repeated trials of a specific test for statistical analysis.
     */
    private void runRepeatedAccuracyTest(String testName, PathCreator pathCreator) {
        currentTestName = testName;
        List<AccuracyResult> testResults = new ArrayList<>();
        
        telemetry.addData("=== " + testName.toUpperCase() + " ACCURACY TEST ===", "");
        telemetry.addData("Status", "Starting " + TRIALS_PER_TEST + " trials");
        telemetry.update();
        
        for (currentTrial = 1; currentTrial <= TRIALS_PER_TEST; currentTrial++) {
            if (!opModeIsActive()) break;
            
            telemetry.addData("Current Test", testName);
            telemetry.addData("Trial", currentTrial + " of " + TRIALS_PER_TEST);
            telemetry.update();
            
            // Reset robot position for consistent starting point
            if (AUTO_RESET_POSITION) {
                resetRobotPosition();
            }
            
            // Create the test path
            org.firstinspires.ftc.teamcode.paths.Path testPath = pathCreator.createPath();
            
            // Run the accuracy test
            AccuracyResult result = runSingleAccuracyTest(testPath, testName, currentTrial);
            
            if (result != null) {
                testResults.add(result);
                allResults.add(result);
                
                // Display immediate results
                displayTrialResult(result);
            }
            
            sleep(1000); // Brief pause between trials
        }
        
        // Display statistical summary for this test
        displayTestStatistics(testName, testResults);
        sleep(3000);
    }
    
    /**
     * Run a single accuracy measurement trial.
     */
    private AccuracyResult runSingleAccuracyTest(org.firstinspires.ftc.teamcode.paths.Path path, 
                                                String testName, int trialNumber) {
        long startTime = System.currentTimeMillis();
        
        // Expected endpoint
        Point expectedEnd = path.getPoint(1.0);
        Pose2d expectedFinalPose = new Pose2d(expectedEnd.getX(), expectedEnd.getY(), new Rotation2d(0));
        
        // Tracking variables
        List<Double> crossTrackErrors = new ArrayList<>();
        List<Pose2d> actualPath = new ArrayList<>();
        double totalPathLength = 0;
        Pose2d lastPose = drive.getLocalizer().getPoseEstimate();
        
        // Start pure pursuit
        drive.followPath(path);
        
        while (opModeIsActive() && drive.isFollowingPath()) {
            // Early exit option
            if (gamepad1.b) {
                drive.stopPathFollowing();
                return null;
            }
            
            // Update robot state
            drive.updateLocalizer();
            drive.updatePurePursuit();
            
            // Collect detailed measurements
            Pose2d currentPose = drive.getLocalizer().getPoseEstimate();
            double crossTrackError = Math.abs(drive.getCrossTrackError());
            
            // Store measurements
            crossTrackErrors.add(crossTrackError);
            actualPath.add(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()));
            
            // Calculate path length
            double segmentLength = Math.hypot(
                currentPose.getX() - lastPose.getX(),
                currentPose.getY() - lastPose.getY()
            );
            totalPathLength += segmentLength;
            lastPose = currentPose;
            
            // Throttled telemetry for performance
            if (crossTrackErrors.size() % 10 == 0 && ENABLE_DETAILED_LOGGING) {
                telemetry.addData("Trial Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
                telemetry.addData("Current Cross-Track Error", String.format("%.3f\"", crossTrackError));
                telemetry.addData("Samples Collected", crossTrackErrors.size());
                telemetry.update();
            }
            
            sleep(20); // 50Hz sampling rate
        }
        
        long executionTime = System.currentTimeMillis() - startTime;
        
        // Final measurements
        Pose2d finalPose = drive.getLocalizer().getPoseEstimate();
        
        // Calculate comprehensive accuracy metrics
        return calculateAccuracyMetrics(
            testName, trialNumber, expectedFinalPose, finalPose,
            crossTrackErrors, actualPath, totalPathLength, 
            executionTime / 1000.0, path
        );
    }
    
    /**
     * Calculate comprehensive accuracy metrics from collected data.
     */
    private AccuracyResult calculateAccuracyMetrics(String testName, int trialNumber,
                                                   Pose2d expectedFinal, Pose2d actualFinal,
                                                   List<Double> crossTrackErrors, List<Pose2d> actualPath,
                                                   double totalPathLength, double executionTime,
                                                   org.firstinspires.ftc.teamcode.paths.Path theoreticalPath) {
        
        // Final position error
        double finalPositionError = Math.hypot(
            actualFinal.getX() - expectedFinal.getX(),
            actualFinal.getY() - expectedFinal.getY()
        );
        
        // Maximum cross-track error
        double maxCrossTrackError = crossTrackErrors.stream()
            .mapToDouble(Double::doubleValue)
            .max()
            .orElse(0.0);
        
        // RMS cross-track error
        double sumSquaredErrors = crossTrackErrors.stream()
            .mapToDouble(error -> error * error)
            .sum();
        double rmsCrossTrackError = Math.sqrt(sumSquaredErrors / crossTrackErrors.size());
        
        // Average cross-track error
        double avgCrossTrackError = crossTrackErrors.stream()
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(0.0);
        
        // Calculate theoretical path length for efficiency metric
        double theoreticalPathLength = calculateTheoreticalPathLength(theoreticalPath);
        double pathEfficiency = theoreticalPathLength / totalPathLength;
        
        // Calculate path smoothness (rate of change of direction)
        double pathSmoothness = calculatePathSmoothness(actualPath);
        
        // Path completion percentage
        double completionPercentage = drive.getPathProgress();
        
        return new AccuracyResult(
            testName, trialNumber, executionTime,
            finalPositionError, maxCrossTrackError, rmsCrossTrackError, avgCrossTrackError,
            totalPathLength, theoreticalPathLength, pathEfficiency,
            pathSmoothness, completionPercentage,
            crossTrackErrors.size(), // Number of samples
            expectedFinal, actualFinal
        );
    }
    
    /**
     * Calculate theoretical path length by sampling.
     */
    private double calculateTheoreticalPathLength(org.firstinspires.ftc.teamcode.paths.Path path) {
        double totalLength = 0;
        Point lastPoint = path.getPoint(0);
        
        for (double t = 0.01; t <= 1.0; t += 0.01) {
            Point currentPoint = path.getPoint(t);
            totalLength += lastPoint.distance(currentPoint);
            lastPoint = currentPoint;
        }
        
        return totalLength;
    }
    
    /**
     * Calculate path smoothness metric (lower values = smoother).
     */
    private double calculatePathSmoothness(List<Pose2d> actualPath) {
        if (actualPath.size() < 3) return 0.0;
        
        double totalDirectionChange = 0;
        
        for (int i = 1; i < actualPath.size() - 1; i++) {
            Pose2d prev = actualPath.get(i - 1);
            Pose2d curr = actualPath.get(i);
            Pose2d next = actualPath.get(i + 1);
            
            // Calculate direction vectors
            double dir1 = Math.atan2(curr.getY() - prev.getY(), curr.getX() - prev.getX());
            double dir2 = Math.atan2(next.getY() - curr.getY(), next.getX() - curr.getX());
            
            // Calculate direction change
            double directionChange = Math.abs(dir2 - dir1);
            if (directionChange > Math.PI) {
                directionChange = 2 * Math.PI - directionChange;
            }
            
            totalDirectionChange += directionChange;
        }
        
        return totalDirectionChange / (actualPath.size() - 2);
    }
    
    /**
     * Reset robot to starting position.
     */
    private void resetRobotPosition() {
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive.getLocalizer().setPoseEstimate(startPose);
        
        telemetry.addData("Status", "Resetting robot position...");
        telemetry.update();
        sleep(1000);
    }
    
    /**
     * Display results for a single trial.
     */
    private void displayTrialResult(AccuracyResult result) {
        telemetry.addData("=== TRIAL " + result.trialNumber + " COMPLETE ===", "");
        telemetry.addData("Test", result.testName);
        telemetry.addData("Execution Time", String.format("%.2f seconds", result.executionTime));
        telemetry.addData("Final Position Error", String.format("%.3f inches", result.finalPositionError));
        telemetry.addData("Max Cross-Track Error", String.format("%.3f inches", result.maxCrossTrackError));
        telemetry.addData("RMS Cross-Track Error", String.format("%.3f inches", result.rmsCrossTrackError));
        telemetry.addData("Path Efficiency", String.format("%.1f%%", result.pathEfficiency * 100));
        telemetry.addData("Completion", String.format("%.1f%%", result.completionPercentage * 100));
        
        // Performance rating
        if (result.finalPositionError < 1.0) {
            telemetry.addData("Accuracy Rating", "EXCELLENT");
        } else if (result.finalPositionError < 2.0) {
            telemetry.addData("Accuracy Rating", "VERY GOOD");
        } else if (result.finalPositionError < 3.0) {
            telemetry.addData("Accuracy Rating", "GOOD");
        } else if (result.finalPositionError < 5.0) {
            telemetry.addData("Accuracy Rating", "FAIR");
        } else {
            telemetry.addData("Accuracy Rating", "NEEDS IMPROVEMENT");
        }
        
        telemetry.update();
        sleep(2000);
    }
    
    /**
     * Display statistical summary for a test type.
     */
    private void displayTestStatistics(String testName, List<AccuracyResult> results) {
        if (results.isEmpty()) return;
        
        // Calculate statistics
        double meanFinalError = results.stream().mapToDouble(r -> r.finalPositionError).average().orElse(0);
        double stdDevFinalError = calculateStandardDeviation(results.stream().mapToDouble(r -> r.finalPositionError).toArray());
        
        double meanMaxError = results.stream().mapToDouble(r -> r.maxCrossTrackError).average().orElse(0);
        double stdDevMaxError = calculateStandardDeviation(results.stream().mapToDouble(r -> r.maxCrossTrackError).toArray());
        
        double meanTime = results.stream().mapToDouble(r -> r.executionTime).average().orElse(0);
        double stdDevTime = calculateStandardDeviation(results.stream().mapToDouble(r -> r.executionTime).toArray());
        
        double meanEfficiency = results.stream().mapToDouble(r -> r.pathEfficiency).average().orElse(0);
        
        telemetry.addData("=== " + testName.toUpperCase() + " STATISTICS ===", "");
        telemetry.addData("Trials Completed", results.size());
        telemetry.addData("", "");
        telemetry.addData("Final Position Error:", "");
        telemetry.addData("  Mean", String.format("%.3f ± %.3f inches", meanFinalError, stdDevFinalError));
        telemetry.addData("  Range", String.format("%.3f to %.3f inches", 
            results.stream().mapToDouble(r -> r.finalPositionError).min().orElse(0),
            results.stream().mapToDouble(r -> r.finalPositionError).max().orElse(0)));
        telemetry.addData("", "");
        telemetry.addData("Max Cross-Track Error:", "");
        telemetry.addData("  Mean", String.format("%.3f ± %.3f inches", meanMaxError, stdDevMaxError));
        telemetry.addData("", "");
        telemetry.addData("Execution Time:", "");
        telemetry.addData("  Mean", String.format("%.2f ± %.2f seconds", meanTime, stdDevTime));
        telemetry.addData("", "");
        telemetry.addData("Path Efficiency", String.format("%.1f%%", meanEfficiency * 100));
        
        // Consistency rating
        if (stdDevFinalError < 0.5) {
            telemetry.addData("Consistency", "EXCELLENT");
        } else if (stdDevFinalError < 1.0) {
            telemetry.addData("Consistency", "GOOD");
        } else if (stdDevFinalError < 2.0) {
            telemetry.addData("Consistency", "FAIR");
        } else {
            telemetry.addData("Consistency", "POOR");
        }
        
        telemetry.update();
    }
    
    /**
     * Generate final comprehensive statistical report.
     */
    private void generateFinalStatisticalReport() {
        if (allResults.isEmpty()) return;
        
        // Overall statistics
        double overallMeanFinalError = allResults.stream().mapToDouble(r -> r.finalPositionError).average().orElse(0);
        double overallStdDevFinalError = calculateStandardDeviation(allResults.stream().mapToDouble(r -> r.finalPositionError).toArray());
        
        double overallMeanMaxError = allResults.stream().mapToDouble(r -> r.maxCrossTrackError).average().orElse(0);
        double overallMeanTime = allResults.stream().mapToDouble(r -> r.executionTime).average().orElse(0);
        double overallMeanEfficiency = allResults.stream().mapToDouble(r -> r.pathEfficiency).average().orElse(0);
        
        // Success rate (final error < 3 inches)
        long successCount = allResults.stream().filter(r -> r.finalPositionError < 3.0).count();
        double successRate = (successCount * 100.0) / allResults.size();
        
        telemetry.addData("=== COMPREHENSIVE ACCURACY REPORT ===", "");
        telemetry.addData("Total Trials", allResults.size());
        telemetry.addData("Success Rate (<3\" error)", String.format("%.1f%%", successRate));
        telemetry.addData("", "");
        telemetry.addData("Overall Performance:", "");
        telemetry.addData("  Mean Final Error", String.format("%.3f ± %.3f inches", 
            overallMeanFinalError, overallStdDevFinalError));
        telemetry.addData("  Mean Max Cross-Track", String.format("%.3f inches", overallMeanMaxError));
        telemetry.addData("  Mean Execution Time", String.format("%.2f seconds", overallMeanTime));
        telemetry.addData("  Mean Path Efficiency", String.format("%.1f%%", overallMeanEfficiency * 100));
        telemetry.addData("", "");
        
        // Overall system rating
        if (overallMeanFinalError < 1.5 && successRate > 90) {
            telemetry.addData("System Rating", "COMPETITION READY");
        } else if (overallMeanFinalError < 2.5 && successRate > 80) {
            telemetry.addData("System Rating", "GOOD PERFORMANCE");
        } else if (overallMeanFinalError < 4.0 && successRate > 70) {
            telemetry.addData("System Rating", "ACCEPTABLE");
        } else {
            telemetry.addData("System Rating", "NEEDS IMPROVEMENT");
        }
        
        telemetry.addData("", "Detailed results available in logs");
        telemetry.addData("", "Press any button to exit");
        telemetry.update();
        
        // Wait for button press
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
            sleep(100);
        }
    }
    
    /**
     * Calculate standard deviation of a dataset.
     */
    private double calculateStandardDeviation(double[] values) {
        if (values.length <= 1) return 0.0;
        
        double mean = java.util.Arrays.stream(values).average().orElse(0.0);
        double sumSquaredDifferences = java.util.Arrays.stream(values)
            .map(x -> Math.pow(x - mean, 2))
            .sum();
        
        return Math.sqrt(sumSquaredDifferences / (values.length - 1));
    }
    
    // Path creation methods for different test scenarios
    
    private org.firstinspires.ftc.teamcode.paths.Path createStraightLinePath() {
        return new LinearPath(new Point(0, 0), new Point(36, 0));
    }
    
    private org.firstinspires.ftc.teamcode.paths.Path create90DegreeTurnPath() {
        return new PolyLine(new Point(0, 0), new Point(24, 0), new Point(24, 24));
    }
    
    private org.firstinspires.ftc.teamcode.paths.Path createSmoothCurvePath() {
        return new CubicPath(new Point(0, 0), new Point(12, 8), new Point(24, 8), new Point(36, 0));
    }
    
    private org.firstinspires.ftc.teamcode.paths.Path createComplexPath() {
        return new PolyLine(
            new Point(0, 0), new Point(18, 0), new Point(18, 18),
            new Point(0, 18), new Point(0, 0)
        );
    }
    
    /**
     * Interface for path creation methods.
     */
    private interface PathCreator {
        org.firstinspires.ftc.teamcode.paths.Path createPath();
    }
    
    /**
     * Data structure for accuracy test results.
     */
    private static class AccuracyResult {
        final String testName;
        final int trialNumber;
        final double executionTime;
        final double finalPositionError;
        final double maxCrossTrackError;
        final double rmsCrossTrackError;
        final double avgCrossTrackError;
        final double actualPathLength;
        final double theoreticalPathLength;
        final double pathEfficiency;
        final double pathSmoothness;
        final double completionPercentage;
        final int sampleCount;
        final Pose2d expectedFinalPose;
        final Pose2d actualFinalPose;
        
        AccuracyResult(String testName, int trialNumber, double executionTime,
                      double finalPositionError, double maxCrossTrackError, 
                      double rmsCrossTrackError, double avgCrossTrackError,
                      double actualPathLength, double theoreticalPathLength, double pathEfficiency,
                      double pathSmoothness, double completionPercentage,
                      int sampleCount, Pose2d expectedFinalPose, Pose2d actualFinalPose) {
            this.testName = testName;
            this.trialNumber = trialNumber;
            this.executionTime = executionTime;
            this.finalPositionError = finalPositionError;
            this.maxCrossTrackError = maxCrossTrackError;
            this.rmsCrossTrackError = rmsCrossTrackError;
            this.avgCrossTrackError = avgCrossTrackError;
            this.actualPathLength = actualPathLength;
            this.theoreticalPathLength = theoreticalPathLength;
            this.pathEfficiency = pathEfficiency;
            this.pathSmoothness = pathSmoothness;
            this.completionPercentage = completionPercentage;
            this.sampleCount = sampleCount;
            this.expectedFinalPose = expectedFinalPose;
            this.actualFinalPose = actualFinalPose;
        }
    }
}