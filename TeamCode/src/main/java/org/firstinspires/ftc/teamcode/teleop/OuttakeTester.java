package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.V4B;

public class OuttakeTester extends LinearOpMode {
    Outtake outtake;
    V4B v4b;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad p1 = new Gamepad();
        outtake = new Outtake(p1, hardwareMap);
        v4b = new V4B(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (p1.cross) {
                v4b.retract();
            } else if (p1.circle){
                v4b.extend();
            }

            telemetry.addLine("V4B Pos: " + v4b.getPosition());

            if (p1.square) {
                outtake.open();
            } else if (p1.triangle) {
                outtake.close();
            }

            telemetry.addLine("Outtake Pos: " + outtake.getPosition());
            telemetry.update();
        }
    }
}
