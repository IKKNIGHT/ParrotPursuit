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
        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // TODO: change the localiser type, to a class that extends localizer
        //   ex : localizer = new Localizer(hardwareMap);
        localizer = new PinpointLocalizer(hardwareMap);
    }
}
