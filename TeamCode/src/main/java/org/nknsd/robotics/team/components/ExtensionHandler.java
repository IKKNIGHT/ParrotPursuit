package org.nknsd.robotics.team.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;

public class ExtensionHandler implements NKNComponent {
    private final String extenderName;
    private final boolean doInvertMotor;
    private final double motorPower;
    private DcMotor motor;          // extender motor
    private RotationHandler rotationHandler;  //Connects to the arm rotator to read from it
    private static final double SAFE_ARM_ROTATION_VALUE = 1.5;

    private ExtensionPositions target = ExtensionPositions.RESTING;

    public enum ExtensionPositions {
        RESTING(0) {
            @Override
            boolean canGoToPosition(RotationHandler rotationHandler) {
                return true;
            }
        },

        COLLECT(1565) {
            @Override
            boolean canGoToPosition(RotationHandler rotationHandler) {
                return rotationHandler.targetRotationPosition == RotationHandler.RotationPositions.PICKUP || rotationHandler.targetRotationPosition == RotationHandler.RotationPositions.PREPICKUP;
            }
        },

        HIGH_BASKET(3100) { //prob closer to 3k
            @Override
            boolean canGoToPosition(RotationHandler rotationHandler) {
                return rotationHandler.targetRotationPosition == RotationHandler.RotationPositions.HIGH;
            }
        };

        final int position;

        ExtensionPositions(int position) {
            this.position = position;
        }

        abstract boolean canGoToPosition(RotationHandler rotationHandler);
    }

    public ExtensionHandler(String extenderName, boolean doInvertMotor, double motorPower) {
        this.extenderName = extenderName;
        this.doInvertMotor = doInvertMotor;
        this.motorPower = motorPower;
    }
    
    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        motor = hardwareMap.dcMotor.get(extenderName);
        if (doInvertMotor) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(motorPower);
        motor.setTargetPosition(ExtensionPositions.RESTING.position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return "ExtensionHandler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("Ext Current Position", motor.getCurrentPosition());
        telemetry.addData("Ext Target Position", motor.getTargetPosition());
        telemetry.addData("Ext State", target.name());
    }

    public boolean gotoPosition(ExtensionPositions extensionPosition) {
        if (extensionPosition.canGoToPosition(rotationHandler)) {
            motor.setTargetPosition(extensionPosition.position);
            target = extensionPosition;
            return true;
        }
        return false;
    }

    public void resetEncoder() {
        if (target == ExtensionPositions.RESTING) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public ExtensionPositions targetPosition() {
        return target;
    }

    public void link(RotationHandler rotationHandler) {
        this.rotationHandler = rotationHandler;
    }

}