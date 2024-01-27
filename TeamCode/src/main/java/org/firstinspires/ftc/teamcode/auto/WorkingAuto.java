package org.firstinspires.ftc.teamcode.auto;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Impasta;

@Autonomous(name="JankAutoWorking", group="ATrashcan")
public class WorkingAuto extends LinearOpMode {
    // Declaring hardware variables
    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Intake;
    private CRServo DRV4BL, DRV4BR;
    private AHRS imu;
    Impasta impasta;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializing motors, servos, and sensors
        fl = hardwareMap.dcMotor.get("leftFront"); // Drivebase
        fr = hardwareMap.dcMotor.get("rightFront"); // Drivebase
        bl = hardwareMap.dcMotor.get("leftRear"); // Drivebase
        br = hardwareMap.dcMotor.get("rightRear"); // Drivebase

        leftSlide = hardwareMap.dcMotor.get("Left Slide"); //Slides
        rightSlide = hardwareMap.dcMotor.get("Right Slide"); //Slides

        Intake = hardwareMap.dcMotor.get("Intake"); //Pixel Intake

        DRV4BL = hardwareMap.crservo.get("V4BL"); //Virtual Four Bar Servos // Left Side
        DRV4BR = hardwareMap.crservo.get("V4BR"); //Virtual Four Bar Servos //Right Side

        imu = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);

        // Creating an instance of the Impasta class
        impasta = new Impasta(fl, fr, bl, br, leftSlide, rightSlide, Intake, imu);

        impasta.reset();
        boolean reset = true;

        boolean DRV4BReset = false;

        telemetry.addLine("Initialization Done, pos reset: " + reset + " DRV4B reset: " + DRV4BReset);
        telemetry.update();

        // Waiting for the start button to be pressed
        waitForStart();

        if (isStopRequested()) return;
        // Main TeleOp loop
        while (opModeIsActive()) {
            while (runtime.seconds() < 3.8 && opModeIsActive()) {
                impasta.driveBaseAuto(0.5 * 1.1);
                telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
                telemetry.update();
            }

            // Driving the robot based on gamepad input
            impasta.driveBaseAuto(0 * 1.1);
        }
    }
}
