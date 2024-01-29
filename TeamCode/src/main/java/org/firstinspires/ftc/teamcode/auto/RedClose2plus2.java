package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;


@Autonomous(name="Red Close 2plus2", group="Close")
public class RedClose2plus2 extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;
    Slides s;
    Outtake o;
    V4B v4b;
    Intake i;
    Gamepad gamepad;
    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    TrajectorySequence prop;
    TrajectorySequence score;
    TrajectorySequence park;

    public static Pose2d startPoseCloseRed = new Pose2d(12,62, Math.toRadians(-90));
    String section;

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        s = new Slides(hardwareMap);
        o = new Outtake(gamepad, hardwareMap);
        v4b = new V4B(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
            telemetry.update();
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        drive.setPoseEstimate(startPoseCloseRed);

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
        TrajectorySequence getPixelAndScore = drive.trajectorySequenceBuilder(scoreCloseMidRed.end())
                .lineToLinearHeading(new Pose2d(46.5, -10.9, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-60, -10.9, Math.toRadians(180)))
                //intake here(code this late)
                .lineToLinearHeading(new Pose2d(46.5, -10.9, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(46.5, -36.3, Math.toRadians(0)))
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

        while (opModeInInit()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length > 0) {
                telemetry.addData("Objects Detected: ", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    if (blocks[i].id == 2) { //If red, then detect
                        telemetry.addData("Object " + (i + 1) + ": ", blocks[i].toString());

                        // Determine which section the object is in based on its X-coordinate
                        int xCoordinate = blocks[i].x;
                        int yCoordinate = blocks[i].y;

                        // Calculate section boundaries
                        int sectionWidth = 320 / 3; // Because horizontal screen resolution is 320
                        int cropHeight = 50;

                        // Determine the section
                        if (xCoordinate < sectionWidth && yCoordinate > cropHeight) {
                            section = "LEFT";
                        } else if (xCoordinate < 2 * sectionWidth && yCoordinate > cropHeight) {
                            section = "MIDDLE";
                        } else {
                            section = "RIGHT";
                        }
                        telemetry.addData("Object Section: ", section);
                    } else { //No objects detected or only red objects detected
                        section = "RIGHT";
                        telemetry.addLine("Default - RIGHT");
                    }
                }
            }
            telemetry.update();
        }

        telemetry.addLine("START - Section is " + section);
        telemetry.update();

        switch(section) {
            case "LEFT":
                prop = propCloseLeftRed;
                score = scoreCloseLeftRed;
                park = parkCloseLeftRed;
                break;
            case "MIDDLE":
                prop = propCloseMidRed;
                score = scoreCloseMidRed;
                park = parkCloseMidRed;
                break;
            default:
                prop = propCloseRightRed;
                score = scoreCloseRightRed;
                park = parkCloseRightRed;
                break;
        }

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
                new TrajectorySequenceCommand(drive, prop),
//                new InstantCommand(() -> {
//                    s.runToPos(Slides.SlidePos.LOW.position);
//                })
                new TrajectorySequenceCommand(drive, score),
                new TrajectorySequenceCommand(drive, park)
        ));
    }
}
