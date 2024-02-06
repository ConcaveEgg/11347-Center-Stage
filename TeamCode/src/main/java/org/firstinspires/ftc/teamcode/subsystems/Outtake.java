package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class    Outtake extends SubsystemBase {

    private static final double L_OPEN_POS = 1;
    private static final double L_CLOSED_POS = 0.9;
    private static final double R_OPEN_POS = 1;
    private static final double R_CLOSED_POS = 0.9;
    private Servo leftOuttake, rightOuttake;
    private Gamepad gamepad;

    public Outtake(Gamepad gamepad, HardwareMap hardwareMap) {
        leftOuttake = hardwareMap.get(Servo.class, "leftOut");
        rightOuttake = hardwareMap.get(Servo.class, "rightOut");

        this.gamepad = gamepad;
    }

    public void open() {
        leftOuttake.setPosition(L_OPEN_POS);
        rightOuttake.setPosition(R_OPEN_POS);
    }

    public void close() {
        leftOuttake.setPosition(L_CLOSED_POS);
        rightOuttake.setPosition(R_CLOSED_POS);
    }

    public void update() {
        if (gamepad.left_trigger > 0.3) {
            leftOuttake.setPosition(L_CLOSED_POS); // left //lower
        } else {
            leftOuttake.setPosition(L_OPEN_POS); // left //raise
        }

        if (gamepad.right_trigger > 0.3) {
            rightOuttake.setPosition(R_CLOSED_POS); // right //lower
        } else {
            rightOuttake.setPosition(R_OPEN_POS); // right //raise
        }
    }
}
