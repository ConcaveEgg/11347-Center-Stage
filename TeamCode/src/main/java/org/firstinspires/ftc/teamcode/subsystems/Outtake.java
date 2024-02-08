package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Outtake extends SubsystemBase {

    public enum outtakePos {
        OPEN(1),
        CLOSE(0.92);

        public double position;

        outtakePos(double position) {
            this.position = position;
        }
    }

    private ServoImplEx leftOuttake, rightOuttake;
    private Gamepad gamepad;
    private DistanceSensor leftSensor, rightSensor;
    private  int distance = 6;

    public Outtake(Gamepad gamepad, HardwareMap hardwareMap) {
        leftOuttake = hardwareMap.get(ServoImplEx.class, "leftOut");
        rightOuttake = hardwareMap.get(ServoImplEx.class, "rightOut");
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightDistance");

        //TODO: Test the following to see if it fixes DC issues of power module
        leftOuttake.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightOuttake.setPwmRange(new PwmControl.PwmRange(500, 2500));

        this.gamepad = gamepad;
    }

    public double returnDistanceLeft() {
        return leftSensor.getDistance(DistanceUnit.CM);
    }
    
    public double returnDistanceRight() {
        return rightSensor.getDistance(DistanceUnit.CM);
    }

    public void detectDistance() {
        if (leftSensor.getDistance(DistanceUnit.CM) < distance) {
            leftOuttake.setPosition(outtakePos.CLOSE.position);
        } else {
            leftOuttake.setPosition(outtakePos.OPEN.position);
        }

        if (rightSensor.getDistance(DistanceUnit.CM) < distance) {
            rightOuttake.setPosition(outtakePos.CLOSE.position);
        } else {
            rightOuttake.setPosition(outtakePos.OPEN.position);
        }
    }

    public void open() {
        leftOuttake.setPosition(outtakePos.OPEN.position);
        rightOuttake.setPosition(outtakePos.OPEN.position);
    }

    public void close() {
        leftOuttake.setPosition(outtakePos.CLOSE.position);
        rightOuttake.setPosition(outtakePos.CLOSE.position);
    }

    public void update() {
        if (gamepad.left_trigger > 0.3) {
            leftOuttake.setPosition(outtakePos.OPEN.position);
        } else {
            leftOuttake.setPosition(outtakePos.OPEN.position);
        }

        if (gamepad.right_trigger > 0.3) {
            rightOuttake.setPosition(outtakePos.CLOSE.position);
        } else {
            rightOuttake.setPosition(outtakePos.CLOSE.position);
        }
    }

    public double getPosition() {
        return leftOuttake.getPosition();
    }
}
