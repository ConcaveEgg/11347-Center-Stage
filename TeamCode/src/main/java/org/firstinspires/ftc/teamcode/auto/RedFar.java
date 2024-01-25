package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


@Autonomous(name="Red Far")
public class RedFar extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;
    Slides s;
    Outtake o;
    V4B v4b;
    GamepadEx gamepad;

    public static Pose2d startPoseFarRed = new Pose2d(12,-62, Math.toRadians(90));

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        s = new Slides(hardwareMap);
        o = new Outtake(gamepad, hardwareMap);
        v4b = new V4B(hardwareMap);


        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting For Start...");
        }

       TrajectorySequence propFarMidRed = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(25)
                .lineToLinearHeading(new Pose2d(-35.6, -32, Math.toRadians(90)))
                .back(6)
                .strafeLeft(15)
                .forward(31.3)
                .build();

        TrajectorySequence propFarLeftRed =  drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(25)
                .lineToLinearHeading(new Pose2d(-36.6, -30.5, Math.toRadians(180)))
                .build();

        TrajectorySequence propFarRightRed = drive.trajectorySequenceBuilder(startPoseFarRed)
                .forward(25)
                .lineToLinearHeading(new Pose2d(-32.6, -30.5, Math.toRadians(0)))
                .build()


        TrajectorySequence scoreFarMidRed = drive.trajectorySequenceBuilder(propFarMidRed.end())
               .forward(81)
               .strafeRight(backBoard-3)
               .build();
//         TrajectorySequence scoreFarRightRed = drive.trajectorySequenceBuilder(propFarRightRed.end())
//                .forward(81)
//                .strafeRight(backBoard+3)
//                .build();

//        TrajectorySecquence parkFarLeftRed = drive.trajectorySequenceBuilder(scoreFarLeftRed.end())
//                .strafeLeft(backBoard-9)
//                .forward(12)
//                .build();

        TrajectorySequence parkFarMidRed = drive.trajectorySequenceBuilder(scoreFarMidRed.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-32.6, -10.5, Math.toRadians(0)))
                .build();

//        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like norma
//            new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                    new TrajectorySequenceCommand(drive, propFarMidRed),
//                    new TrajectorySequenceCommand(drive, scoreFarMidRed),
//                    new TrajectorySequenceCommand(drive, parkFarMidRed)
//                ),
//                new InstantCommand(() -> {
//                    new WaitCommand(4000);
//                    s.goToPosition(Slides.SlidePos.LOW);
////                    new WaitCommand(2000);
////                    v4b.togglePower();
////                    new WaitCommand(2000);
////                    o.open();
////                    new WaitCommand(2000);
////                    o.Close();
////                    v4b.togglePower();
////                    new WaitCommand(1000);
////                    s.goToPosition(Slides.SlidePos.DOWN);
//                })
//            )
//        ));

        schedule(new SequentialCommandGroup(
//                new TrajectorySequenceCommand(drive, propCFarMidRed),
                new InstantCommand(() -> {
                        s.runToPos(Slides.SlidePos.LOW.position);
                })
//                new TrajectorySequenceCommand(drive, scoreFarMidRed),
//                new TrajectorySequenceCommand(drive, parkFarMidRed)
        ));
    }
}