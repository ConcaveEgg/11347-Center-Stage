package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Mechanisms extends SubsystemBase {
    Impasta impasta;
    private Gamepad gamepad1, gamepad2;
    private Servo out1, out2, launchPlane;
    private Servo DRV4BL, DRV4BR;
    //    private CRServo DRV4BL, DRV4BR;
    private DistanceSensor leftSensor, rightSensor;
    //Need correct value
    private  int distance = 3;
    private  boolean leftOuttakeDown, rightOuttakeDown;

    public Mechanisms(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap) {
//        DRV4BL = hardwareMap.crservo.get("leftV4B"); //Virtual Four Bar Servos // Left Side
//        DRV4BR = hardwareMap.crservo.get("rightV4B"); //Virtual Four Bar Servos //Right Side
        DRV4BL = hardwareMap.servo.get("leftV4B");
        DRV4BR = hardwareMap.servo.get("rightV4B");
        launchPlane = hardwareMap.servo.get("launcher");
        out1 = hardwareMap.servo.get("leftOut"); //Outtake
        out2 = hardwareMap.servo.get("rightOut"); //Outtake
        leftSensor = hardwareMap.get(DistanceSensor.class, "Left Sensor");
        rightSensor = hardwareMap.get(DistanceSensor.class, "Right Sensor");


        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void launch() {
        launchPlane.setPosition(0);
    }

    public void resetLaunch() {
        launchPlane.setPosition(1);
    }

//    public void v4b() {
//        if (gamepad2.circle) {
//            // Rotate the CR servos clockwise as long as the button is pressed
//            while (gamepad2.circle) {
//                DRV4BL.setPower(1.0);  // Adjust the power as needed
//                DRV4BR.setPower(1.0);  // Adjust the power as needed
//            }
//            // Stop the servos when the button is released
//            DRV4BL.setPower(0);
//            DRV4BR.setPower(0);
//
//        } else if (gamepad2.cross) {
//            // Rotate the CR servos counterclockwise as long as the button is pressed
//            while (gamepad2.cross) {
//                DRV4BL.setPower(-1.0);  // Adjust the power as needed
//                DRV4BR.setPower(-1.0);  // Adjust the power as needed
//            }
//            // Stop the servos when the button is released
//            DRV4BL.setPower(0);
//            DRV4BR.setPower(0);
//
//        }
//    }

    public void extend() {
        DRV4BL.setPosition(0.45);
        DRV4BR.setPosition(0.45);
    }

    public void retract() {
        DRV4BL.setPosition(0.01);
        DRV4BR.setPosition(0.01);
    }
//  TODO: Test distance sensors
    public void outtakes() {
        if (gamepad2.left_trigger > 0.3 || leftSensor.getDistance(DistanceUnit.INCH) <= distance) {
            gamepad2.rumble(1000);
            out1.setPosition(1);
            if (gamepad2.left_trigger > 0.3) {
                leftOuttakeDown = true;
            }
        }
        if (leftOuttakeDown && gamepad2.left_trigger <= 0.3) {
            out1.setPosition(0.92);
            leftOuttakeDown = false;
        }

        if (gamepad2.right_trigger > 0.3 || rightSensor.getDistance(DistanceUnit.INCH) <= distance) {
            gamepad2.rumble(1000);
            out2.setPosition(1);
            if (gamepad2.right_trigger > 0.3) {
                rightOuttakeDown = true;
            }
        }
        if (rightOuttakeDown && gamepad2.right_trigger <= 0.3) {
            out2.setPosition(0.92);
            rightOuttakeDown = false;
        }
    }
}
