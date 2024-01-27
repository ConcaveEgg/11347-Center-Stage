package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mechanisms extends SubsystemBase {
    Impasta impasta;
    private Gamepad gamepad1, gamepad2;
    private Servo out1, out2, launchPlane, DRV4BL, DRV4BR;

    public Mechanisms(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap) {
        DRV4BL = hardwareMap.servo.get("leftV4B"); //Virtual Four Bar Servos // Left Side
        DRV4BR = hardwareMap.servo.get("rightV4B"); //Virtual Four Bar Servos //Right Side
        launchPlane = hardwareMap.servo.get("launcher");
        out1 = hardwareMap.servo.get("leftOut"); //Outtake
        out2 = hardwareMap.servo.get("rightOut"); //Outtake

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void airplaneLauncher() {
        if (gamepad2.triangle) {
            launchPlane.setPosition(0);
        } else if (gamepad2.square) {
            launchPlane.setPosition(1);
        }
    }

    public void v4b() {
        if (gamepad2.circle) {
            DRV4BL.setPosition(0.65);
            DRV4BR.setPosition(0.65);
        } else if (gamepad2.cross) {
            DRV4BL.setPosition(0);
            DRV4BR.setPosition(0);
        }
    }

    public void outtakes() {
        if (gamepad2.left_trigger > 0.3 /*|| DetectDistance() <= distance*/) {
//            gamepad2.rumble(100);
            out1.setPosition(0.75); // left //lower
        } else {
            out1.setPosition(0.55); // left //raise
        }

        if (gamepad2.right_trigger > 0.3 /*|| DetectDistance() <= distance */) {
//            gamepad2.rumble(100);
            out2.setPosition(0.5); // right //lower
        } else {
            out2.setPosition(0.6); // right //raise
        }
    }
}
