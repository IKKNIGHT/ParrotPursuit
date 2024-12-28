package org.firstinspires.ftc.teamcode.tuning.otos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.localization.SparkFunOTOSLocalizer;

@TeleOp
public class OTOSAngularScalarTuner extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSLocalizer drive = new SparkFunOTOSLocalizer(hardwareMap);
        double radsTurned = 0;
        Rotation2d heading = new Rotation2d(0);
        Rotation2d lastHeading = new Rotation2d(0);
        telemetry.addLine("OTOS Angular Scalar Tuner");
        telemetry.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        telemetry.addLine("Then copy the scalar into DriveConstants.SparkFunOTOSConfig.ANGULAR_SCALAR.");
        packet.addLine("OTOS Angular Scalar Tuner");
        packet.addLine("Press START, then rotate the robot on the ground 10 times (3600 degrees).");
        packet.addLine("Then copy the scalar into DriveConstants.SparkFunOTOSConfig.ANGULAR_SCALAR.");
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            heading = new Rotation2d(drive.getPoseEstimate().getHeading());
            radsTurned += heading.minus(lastHeading).getRadians();
            lastHeading = heading;
            telemetry.addData("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            telemetry.addData("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned));
            packet.put("Uncorrected Degrees Turned", Math.toDegrees(radsTurned));
            packet.put("Calculated Angular Scalar", 3600 / Math.toDegrees(radsTurned));
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
