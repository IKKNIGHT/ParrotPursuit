package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Commands.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.SlideTargetHeight;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class LeftSideAuto87Pts extends SequentialCommandGroup {

    // constructor
    public LeftSideAuto87Pts() {
        // start pos (0.25, 1.6, -90) on field
        addCommands (
                new InstantCommand(() -> RobotContainer.odometry.setCurrentPos(new Pose2d(0.25, 1.6, new Rotation2d(Math.toRadians(-90))))),
                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 1.0, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                new SpecimenPlacePos(),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW)),

                new Pause(2),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 0.85, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                new Pause(2),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMLE_SPECIMEN)),

                new Pause(3),

                new OpenClaw(),

                new Pause(1),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO)),

                new Pause(3),

                new CloseClaw(),

                new Pause(1),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 1.15, new Rotation2d(Math.toRadians(90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                new HuntingPos(),

                new OpenClaw(),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 0.8, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                // use camera to orient to specimen
                // claw will close using touch sensor when it touches a sample
                new DropToGrab(),

                new Pause(2),

                new CloseClaw(),

                new Pause(2),

                new HuntingPos(),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 1.15, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                new Pause(2),

                new ArmStowHigh(),

                new Pause(2),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(1.38, 1.38, new Rotation2d(Math.toRadians(-135))),
                        new Rotation2d(Math.toRadians(-135))),

                new Pause(2),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new Pause(2),

                new BackDepositePose(),

                new Pause(5),
                //new OpenClaw(),

                new ArmStowHigh(),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO)),

                new Pause(2),
//Finish Later
                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(1.38, 1.38, new Rotation2d(Math.toRadians(-135))),
                        new Rotation2d(Math.toRadians(-135)))


        );


    }

}

//                Example
//                new FollowPath(
//                        2.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(90.0)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(1.45, 1.45, new Rotation2d(Math.toRadians(45.0))),
//                        new Rotation2d(Math.toRadians(-135.0))

// Example #1: Lily's 2023 FRC super cube auto
/*          // enable arm, and lift to stow position
            new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

            // move arm back to drop off cube
            new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

            // delay until arm gets back
            new DelayCommand(1.0),

            // place cube
            new InstantCommand(() -> RobotContainer.grabber.setClose()),

            // delay for gripper to close
            new DelayCommand(0.7),

            // move arm to 'forward position' but inside robot bumper)
            // move to 135deg
            new InstantCommand(() -> RobotContainer.arm.SetArmPosition(135.0)),

            // delay for arm to get to stow
            new DelayCommand(1.5),

            // ensure arm is stowed before it is allow to begin moving over charge station
            new SafetyCheckStowPosition(),

            // drive right
            // new DrivetoRelativePose(new Pose2d(0,-2.0, new Rotation2d(0.0)), 1.0, 0.1, 5.0),

            // drive straight
            new DrivetoRelativePose(new Pose2d(5.0, 0, new Rotation2d(0.0)),1.8,0.1, 7.0),

            // pick up cube from floor :)
            new AutoFloorCubePickup(),

            // delay
            new DelayCommand(0.5),

            // drive back
            //new DrivetoRelativePose(new Pose2d(1.0,0, new Rotation2d(0.0)), 1.0, 0.1, 2.0),

            // drive left to center
            new DrivetoRelativePose(new Pose2d(-1.0,2.0, new Rotation2d(0.0)), 1.8, 0.1, 5.0),

            // drive straight onto charge station
            new DrivetoRelativePose(new Pose2d(-1.5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),

            // balance
            new AutoBalance()

// Example #2: Matthew's 2024 shoot donut sequence.
This sequence contains parallel and parallelrace subgroups within an overall series command

      addCommands(
      new ParallelRaceGroup(
        new AimToSpeaker(),
        new SpinupSpeaker()
      ),
      new ParallelCommandGroup(
        new WaitForEffectorAngle(),
        new WaitForShooterSpinup()
      ),

      new ShootSpeaker()
    );

 */