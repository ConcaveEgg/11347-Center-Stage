package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Launcher extends SubsystemBase {

    private Servo launchServo;
    private Gamepad gamepad;

    public Launcher(Gamepad gamepad, HardwareMap hardwareMap) {
        launchServo = hardwareMap.get(Servo.class, "launch");

        this.gamepad = gamepad;
    }

    public void airplaneLauncher() {
        if (gamepad.triangle) {
            launchServo.setPosition(0);
        } else if (gamepad.square) {
            launchServo.setPosition(1);
        }
    }
}
