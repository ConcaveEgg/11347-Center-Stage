package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Impasta;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.V4B;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;


@Autonomous(name="Blue Close", group="Close")
public class BlueClose extends CommandOpMode {
    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;
    Slides s;
    Outtake o;
    V4B v4b;
    Gamepad gamepad;

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    TrajectorySequence prop;
    TrajectorySequence score;
    TrajectorySequence park;

    public static Pose2d startPoseCloseBlue = new Pose2d(12,62, Math.toRadians(-90));
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

        drive.setPoseEstimate(startPoseCloseBlue);

        TrajectorySequence propCloseLeftBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(25)
                .turn(Math.toRadians(90))
                .forward(11)
                .back(11)
                .strafeLeft(25)
                .build();

        TrajectorySequence scoreCloseLeftBlue = drive.trajectorySequenceBuilder(propCloseLeftBlue.end())
                .forward(34)
                .strafeRight(12)
                .build();

        TrajectorySequence parkCloseLeftBlue = drive.trajectorySequenceBuilder(scoreCloseLeftBlue.end())
                .strafeLeft(12)
                .forward(12)
                .build();

        TrajectorySequence propCloseMidBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(25)
                .lineToLinearHeading(new Pose2d(12, 62, 0))
                .build();

        TrajectorySequence scoreCloseMidBlue = drive.trajectorySequenceBuilder(propCloseMidBlue.end())
                .forward(34)
                .strafeRight(19)
                .build();

        TrajectorySequence parkCloseMidBlue = drive.trajectorySequenceBuilder(scoreCloseMidBlue.end())
                .strafeLeft(25)
                .forward(12)
                .build();

        TrajectorySequence propCloseRightBlue = drive.trajectorySequenceBuilder(startPoseCloseBlue)
                .forward(25)
                .turn(Math.toRadians(-90))
                .forward(11)
                .back(11)
                .lineToLinearHeading(new Pose2d(12, 62, 0))
                .build();

        TrajectorySequence scoreCloseRightBlue = drive.trajectorySequenceBuilder(propCloseRightBlue.end())
                .forward(34)
                .strafeRight(12)
                .build();

        TrajectorySequence parkCloseRightBlue = drive.trajectorySequenceBuilder(scoreCloseRightBlue.end())
                .strafeLeft(24)
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
                    if (blocks[i].id == 1) { //If blue, then detect
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
            } else {
                section = "MIDDLE";
            }
            telemetry.update();
        }

        telemetry.addLine("START - Section is " + section);
        telemetry.update();

        switch(section) {
            case "LEFT":
                prop = propCloseLeftBlue;
                score = scoreCloseLeftBlue;
                park = parkCloseLeftBlue;
                break;
            case "MIDDLE":
                prop = propCloseMidBlue;
                score = scoreCloseMidBlue;
                park = parkCloseMidBlue;
                break;
            default:
                prop = propCloseRightBlue;
                score = scoreCloseRightBlue;
                park = parkCloseRightBlue;
                break;
        }

//        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like norma
//            new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                    new TrajectorySequenceCommand(drive, propCloseMidBlue),
//                    new TrajectorySequenceCommand(drive, scoreCloseMidBlue),
//                    new TrajectorySequenceCommand(drive, parkCloseMidBlue)
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
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new TrajectorySequenceCommand(drive, propCloseLeftBlue),
                                new TrajectorySequenceCommand(drive, score),
                                new TrajectorySequenceCommand(drive, park)
                        )
//                    new InstantCommand(() -> {
//                  s.runToPos(Slides.SlidePos.LOW.position);
//                })
                )

        ));
    }
}
