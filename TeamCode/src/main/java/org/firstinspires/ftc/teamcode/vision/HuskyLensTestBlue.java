package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Sensor: HuskyLens-Blue", group = "Sensor")
public class HuskyLensTestBlue extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length == 1) {
                telemetry.addData("Objects Detected: ", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    if (blocks[i].id == 1) {
                        telemetry.addData("Object " + (i + 1) + ": ", blocks[i].toString());

                        // Determine which section the object is in based on its X-coordinate
                        int xCoordinate = blocks[i].x;
                        int yCoordinate = blocks[i].y;

                        // Calculate section boundaries
                        int sectionWidth = 320 / 3; // Because horizontal screen resolution is 320
                        int cropHeight = 50;

                        // Determine the section
                        String section;
                        if (xCoordinate < sectionWidth && yCoordinate > cropHeight) {
                            section = "LEFT";
                        } else if (xCoordinate < 2 * sectionWidth && yCoordinate > cropHeight) {
                            section = "Middle";
                        } else {
                            section = "RIGHT";
                        }
                        telemetry.addData("Object Section: ", section);
                    }
                }
            } else {
                telemetry.addLine("RIGHT");
            }

            telemetry.update();
        }
    }
}
