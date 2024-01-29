package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class V4B extends SubsystemBase {

    public enum V4BState {
        RETRACT(0.65),
        EXTEND(0.0);

        public double position;

        V4BState(double power) {
            this.position = power;
        }
    }

    private CRServo servo1, servo2;

    public V4B(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "leftV4B"); // Left Side
        servo2 = hardwareMap.get(CRServo.class, "rightV4B"); // Right Side

    }

    @Override
    public void periodic() {}

//    public double getPosition() {
//        return servo1.getPosition();
//    }

    public void forceDown() {
        servo1.setPower(V4BState.RETRACT.position);
        servo2.setPower(V4BState.RETRACT.position);
    }

    public void setPosition(V4BState state) {
        servo1.setPower(state.position);
        servo2.setPower(state.position);
    }

    public double getPower() {
        return servo1.getPower();
    }

    public void togglePower() {
        if (getPower() == V4BState.RETRACT.position) {
            setPosition(V4BState.EXTEND);
        } else {
            setPosition(V4BState.RETRACT);
        }
    }
}
