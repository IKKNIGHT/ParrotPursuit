package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;
//cock warren like
@Autonomous(name= "BlueLeftP", preselectTeleOp = "OneGPTeleop")
public class BlueLeftP extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.VSLAM, false, false);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        int position = robot.RedElemTest(this,0,0);
        robot.setPosition(0,0,0);
        waitForStart();
        robot.goToPosition(0,-13,0,0,0.5);
        //Turret extension combined with rotation in such a way to achieve the current location to drop the loaded freight into the correct position by barcode
        if(position==2) {
            robot.TurretSlidesToPosition(.75, 1.2, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.75);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==0){
            robot.TurretSlidesToPosition(4.0, 4.0, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.6);
            sleep(400);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        if(position==1){
            robot.TurretSlidesToPosition(7.5, 9.0, 0, 0.5);
            sleep(1300);
            robot.FlipBasketArmToPosition(.45);
            sleep(500);
            robot.FlipBasketToPosition(0.0);
            sleep(1000);
        }
        robot.FlipBasketToPosition(0.4);
        robot.TurretSlidesToPosition(0,0,0,0.5);
        sleep(1500);
        robot.FlipBasketArmToPosition(0.00);
        robot.goToPosition(1,-5,-6,-100,0.5);
        robot.partOfPolySplineToPositionHead(0,-6,-10,-6,-10,5,-7, 7,-5,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0,-6,-10,5,-7, 7,-5,25,-5,true,true,0.5);
        robot.partOfPolySplineToPositionHead(0,5,-7, 7,-5,25,-5, 50,-5,true,true,0.5);
        robot.turnInPlace(-90,0.5);
        robot.goToPosition(0,-6,32,-90,0.5);
        stop();
    }
}