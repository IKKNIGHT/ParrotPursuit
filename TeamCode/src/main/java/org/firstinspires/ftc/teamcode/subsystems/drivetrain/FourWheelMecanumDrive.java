package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

/**
 * for wheel macanum drive train that hold references to 4 wheels and init them.
 */
public class FourWheelMecanumDrive extends BasicDriveTrain {

    protected Motor fL, fR, bL, bR;

    public FourWheelMecanumDrive(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        super(hardwareMap, gamepad, telemetry, feedback);
    }

    @Override
    protected void createAndInitHardwares(HardwareMap hardwareMap) {
        this.fL = new Motor(hardwareMap, "FL");
        this.bL = new Motor(hardwareMap, "BL");
        this.fR = new Motor(hardwareMap, "FR");
        this.bR = new Motor(hardwareMap, "BR");

        fL.encoder.reset();
        fR.encoder.reset();
        bL.encoder.reset();
        bR.encoder.reset();

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    protected void createDrive() {
        this.drive = new MecanumDrive(this.fL, this.fR, this.bL, this.bR);
    }

    public Motor getfL() {
        return fL;
    }

    public Motor getfR() {
        return fR;
    }

    public Motor getbL() {
        return bL;
    }

    public Motor getbR() {
        return bR;
    }
}