package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.UpdateAction;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathChain;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.utils.PathScheduler;

import java.util.List;

/**
 * Simple test OpMode for Pure Pursuit - just follows a straight line
 * Use this for initial testing and tuning
 */
@Autonomous(name = "Pure Pursuit Simple Test", group = "Testing")
public class SimplePurePursuitTest extends LinearOpMode {
    
    static MecanumDrive drive;
    PathScheduler pathScheduler;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system at origin
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Test", "Will drive forward 36x36 inches using Pure Pursuit");
        telemetry.addData("Instructions", "Press play to start");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {

            LinearPath a = new LinearPath(
                    new Point(drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY()),
                    new Point(36, 0)     // Strafe 36 inches
            );

            LinearPath b = new LinearPath(
                    new Point(36, 0),     // Start at origin
                    new Point(36, 36)     // Strafe 36 inches
            );

            pathScheduler = new PathScheduler(List.of(a,b), drive);
            
            telemetry.addData("Status", "Starting Pure Pursuit");
            telemetry.update();

            // Main control loop
            while (opModeIsActive() && drive.isFollowingPath()) {
                pathScheduler.runScheduledPaths(opModeIsActive(), new TelemetryAction(telemetry));
                // alternatively
                drive.followPath(a);
                drive.updatePurePursuit();
                drive.updateLocalizer();
            }
            
            // Path completed
            telemetry.addData("Status", "Path Complete!");
            telemetry.update();
        }

    }
    public static class TelemetryAction extends UpdateAction{

        private Telemetry telemetry;

        public TelemetryAction(Telemetry telemetry){
            this.telemetry = telemetry;
        }

        @Override
        public void onUpdate() {
            // do ur update stuff here
            telemetry.addData("Status", "Following Path");
            telemetry.addData("Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
            telemetry.addData("Position", String.format("(%.1f, %.1f)",
                    drive.getLocalizer().getPoseEstimate().getX(),
                    drive.getLocalizer().getPoseEstimate().getY()));
            telemetry.update();
        }
    }

}