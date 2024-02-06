package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private DcMotorEx intakeMotor;
    private Gamepad gamepad;
    private boolean isAuto;

    public Intake(Gamepad gamepad, HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        isAuto = false;

        this.gamepad = gamepad;
    }

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        isAuto = true;
    }

    public void intake() {
        if (!isAuto) {
            double power = gamepad.left_trigger - gamepad.right_trigger;
            intakeMotor.setPower(power);
        } else {
            intakeMotor.setPower(1);
        }
    }
    public void autoIntake(double p) {
        intakeMotor.setPower(p);
    }

    public void outtake() {
        intakeMotor.setPower(-1);
    }

    public void forceStop() {
        intakeMotor.setPower(0);
    }
}
