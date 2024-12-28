package org.firstinspires.ftc.teamcode.tuning.otos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.SparkFunOTOSLocalizer;

@TeleOp(name="OTOS Position Offset Tuner", group="Tuning")
public class OTOSPositionOffsetTuner extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() throws InterruptedException {
        SparkFunOTOSLocalizer drive = new SparkFunOTOSLocalizer(hardwareMap);
        telemetry.addLine("OTOS Position Offset Tuner");
        telemetry.addLine("Line the robot against the corner of two walls facing forward and Press START.");
        telemetry.addLine("Then rotate the robot exactly 180 degrees and press it back into the corner.");
        telemetry.addLine("Finally, copy the pose offset into DriveConstants.SparkFunOTOSConfig.OFFSET.x/y");
        packet.addLine("OTOS Position Offset Tuner");
        packet.addLine("Line the robot against the corner of two walls facing forward and Press START.");
        packet.addLine("Then rotate the robot exactly 180 degrees and press it back into the corner.");
        packet.addLine("Finally, copy the pose offset into DriveConstants.SparkFunOTOSConfig.OFFSET.x/y");
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("Heading (deg)",Math.toDegrees(drive.getPoseEstimate().getHeading()));
            if (Math.abs(Math.toDegrees(drive.getPoseEstimate().getHeading())) > 175) {
                telemetry.addData("X Offset", drive.getPoseEstimate().getX() / 2);
                telemetry.addData("Y Offset", drive.getPoseEstimate().getY() / 2);
            } else {
                telemetry.addLine("Rotate the robot 180 degrees and align it to the corner again.");
            }
            telemetry.update();
        }
    }
}
