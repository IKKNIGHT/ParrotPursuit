package org.firstinspires.ftc.teamcode.tuning.otos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.SparkFunOTOSLocalizer;

@TeleOp(name="OTOS Heading Offset Tuner", group="Tuning")
public class OTOSHeadingOffsetTuner extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSLocalizer drive = new SparkFunOTOSLocalizer(hardwareMap);
        telemetry.addLine("OTOS Heading Offset Tuner");
        telemetry.addLine("Line the side of the robot against a wall and Press START.");
        telemetry.addLine("Then push the robot forward some distance.");
        telemetry.addLine("Finally, copy the heading offset into DriveConstants.SparkFunOTOSConfig.OFFSET.h");
        packet.addLine("OTOS Heading Offset Tuner");
        packet.addLine("Line the side of the robot against a wall and Press START.");
        packet.addLine("Then push the robot forward some distance.");
        packet.addLine("Finally, copy the heading offset into DriveConstants.SparkFunOTOSConfig.OFFSET.h");
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("Heading Offset (radians, enter this one into DriveConstants.SparkFunOTOSConfig.OFFSET.h!)",Math.atan2(drive.getPoseEstimate().getY(),drive.getPoseEstimate().getX()));
            telemetry.addData("Heading Offset (degrees)",Math.toDegrees(Math.atan2(drive.getPoseEstimate().getY(), drive.getPoseEstimate().getX())));
            packet.put("Heading Offset (radians, enter this one into DriveConstants.SparkFunOTOSConfig.OFFSET.h!)",Math.atan2(drive.getPoseEstimate().getY(),drive.getPoseEstimate().getX()));
            packet.put("Heading Offset (degrees)",Math.toDegrees(Math.atan2(drive.getPoseEstimate().getY(), drive.getPoseEstimate().getX())));
            telemetry.update();
        }
    }
}
