package org.firstinspires.ftc.teamcode.src.DrivePrograms.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

/**
 * The Autonomous ran on Red side near Warehouse for Meet 3
 */
@Autonomous(name = "Red Warehouse Autonomous")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        MarkerPosition Pos = MarkerPosition.NotSeen;
        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
        this.initVuforia();
        this.initTfod();
        this.activateTF();

        odometry.setPosition(7, 63, 90);

        while (!isStarted() && !isStopRequested()) {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        }


        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            System.gc();


            switch (Pos) {
                case NotSeen:
                    telemetry.addData("position", " is far right ");
                    telemetry.update();

                    driveSystem.moveToPosition(20, 84, 1);
                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);

                    Thread.sleep(1500);


                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 81, 1);
                    intake.setServoDown();
                    Thread.sleep(500);


                    break;
                case Right:
                    telemetry.addData("position", " is center");
                    telemetry.update();
                    driveSystem.moveToPosition(20, 85, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);

                    Thread.sleep(1500);

                    intake.setServoDown();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 84, 1);


                    break;
                case Left:
                    telemetry.addData("position", "is left");
                    telemetry.update();
                    driveSystem.moveToPosition(20, 85, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);

                    Thread.sleep(1500);

                    intake.setServoDown();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 84, 1);


                    break;


            }
            //Shared Code
            intake.intakeOn();
            Thread.sleep(1000);
            intake.intakeOff();
            driveSystem.moveToPosition(23, 84, 1);
            intake.setServoUp();
            slide.setTargetLevel(LinearSlide.HeightLevel.Down);
            Thread.sleep(500);
            //following this is unique to carousel and warehouse

            driveSystem.turnTo(170, .5);

            driveSystem.moveToPosition(10, 63, 1);
            driveSystem.strafeAtAngle(90, .5);
            Thread.sleep(300);

            driveSystem.moveToPosition(9, 24, 1);

        }
        slide.end();
        odometry.end();

    }
}


