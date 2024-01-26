package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Impasta;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


@Autonomous(name="Red Close")
public class RedClose extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;
    Impasta bot;
    GamepadEx gamepad;

    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Winch, Intake;
    private Servo out1, out2, launchPlane, aimLauncher;
    private Servo DRV4BL, DRV4BR;

    public static Pose2d startPoseCloseRed = new Pose2d(12,-62, Math.toRadians(90));

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        fl = hardwareMap.dcMotor.get("leftFront"); //Drivebase
        fr = hardwareMap.dcMotor.get("rightFront"); //Drivebase
        bl = hardwareMap.dcMotor.get("leftRear"); //Drivebase
        br = hardwareMap.dcMotor.get("rightRear"); //Drivebase

        leftSlide = hardwareMap.dcMotor.get("frontEncoder"); //Slides
        rightSlide = hardwareMap.dcMotor.get("Right Slide"); //Slides

        Intake = hardwareMap.dcMotor.get("leftEncoder"); //Pixel Intake

        DRV4BL = hardwareMap.servo.get("leftV4B"); //Virtual Four Bar Servos // Left Side
        DRV4BR = hardwareMap.servo.get("rightV4B"); //Virtual Four Bar Servos //Right Side

        launchPlane = hardwareMap.servo.get("launcher");

        out1 = hardwareMap.servo.get("leftOut"); //Outtake
        out2 = hardwareMap.servo.get("rightOut"); //Outtake

        bot = new Impasta(fl, fr, bl, br, leftSlide, rightSlide, Intake, out1, out2, DRV4BL, DRV4BR);


        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting For Start...");
        }

        TrajectorySequence propCloseLeftRed = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(25)
                .turn(Math.toRadians(90))
                .forward(11)
                .back(11)
                .lineToLinearHeading(new Pose2d(12, -62, 0))
                .build();
        TrajectorySequence scoreCloseLeftRed = drive.trajectorySequenceBuilder(propCloseLeftRed.end())
                .forward(34)
                .strafeLeft(24)
                .build();

        TrajectorySequence parkCloseLeftRed = drive.trajectorySequenceBuilder(scoreCloseLeftRed.end())
                .strafeRight(24)
                .forward(12)
                .build();;

        TrajectorySequence propCloseMidRed = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(25)
                .lineToLinearHeading(new Pose2d(12, -62, 0))
                .build();

        TrajectorySequence scoreCloseMidRed = drive.trajectorySequenceBuilder(propCloseMidRed.end())
                .forward(34)
                .strafeLeft(19)
                .build();

        TrajectorySequence parkCloseMidRed = drive.trajectorySequenceBuilder(scoreCloseMidRed.end())
                .strafeRight(25)
                .forward(12)
                .build();

        TrajectorySequence propCloseRightRed = drive.trajectorySequenceBuilder(startPoseCloseRed)
                .forward(25)
                .turn(Math.toRadians(-90))
                .forward(11)
                .back(11)
                .strafeRight(25)
                .build();

        TrajectorySequence scoreCloseRightRed = drive.trajectorySequenceBuilder(propCloseRightRed.end())
                .forward(34)
                .strafeLeft(12)
                .build();

        TrajectorySequence parkCloseRightRed = drive.trajectorySequenceBuilder(scoreCloseRightRed.end())
                .strafeRight(12)
                .forward(12)
                .build();

//        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like norma
//            new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                    new TrajectorySequenceCommand(drive, propCloseMidRed),
//                    new TrajectorySequenceCommand(drive, scoreCloseMidRed),
//                    new TrajectorySequenceCommand(drive, parkCloseMidRed)
//                ),
//                new InstantCommand(() -> {
//                    new WaitCommand(4000);
//                    s.goToPosition(Slides.SlidePos.LOW);
////                    new WaitCommand(2000);
////                    v4b.togglePower();
////                    new WaitCommand(2000);
////                    o.open();
////                    new WaitCommand(2000);
////                    o.close();
////                    v4b.togglePower();
////                    new WaitCommand(1000);
////                    s.goToPosition(Slides.SlidePos.DOWN);
//                })
//            )
//        ));

        schedule(new SequentialCommandGroup(
//                new TrajectorySequenceCommand(drive, propCloseMidRed),
                new InstantCommand(() -> {
                })
//                new TrajectorySequenceCommand(drive, scoreCloseMidRed),
//                new TrajectorySequenceCommand(drive, parkCloseMidRed)
        ));
    }
}