package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;

public class MecanumDrive {

    DcMotorEx leftFront,leftBack,rightBack,rightFront;
    Localizer localizer;
    public Pose2d pose;
    public static class Params{
        // params here... pid etc...
    }
    public static Params PARAMS = new Params();
    /**
     * Constructs MecanumDrive.
     *
     * @param hardwareMap hardwaremap variable from the opmode.
     * @param pose Starting Pose of the robot.
     */
    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose){
        this.pose = pose;


        // change the config in the DriveConstants.java class
        leftFront = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.LEFT_BACK);
        rightBack = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.RIGHT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, DriveConstants.MotorConfig.RIGHT_FRONT);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // add more localizer classes as you wish here.
        if(DriveConstants.LOCALIZER_CLASS == PinpointLocalizer.class){
            localizer = new PinpointLocalizer(hardwareMap);
        }
    }
}
