package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Impasta;
import org.firstinspires.ftc.teamcode.subsystems.Mechanisms;
import org.firstinspires.ftc.teamcode.subsystems.SlidePID;


@TeleOp
public class ImpastaCommandTeleOp extends CommandOpMode {

    // Declaring hardware variables
    Impasta impasta;
    Mechanisms mechanisms;

    //Slide Positions
    final int HIGH = 130;
    final int MID = 90;
    final int LOW = 70;
    final int GROUND = 50;

    private boolean pidActive = false;
    private boolean nothing = false;

    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Intake;

    @Override
    public void initialize() {
        // Initializing hardware
        GamepadEx mechanism = new GamepadEx(gamepad2);

        fl = hardwareMap.dcMotor.get("leftFront"); // Drivebase
        fr = hardwareMap.dcMotor.get("rightFront"); // Drivebase
        bl = hardwareMap.dcMotor.get("leftRear"); // Drivebase
        br = hardwareMap.dcMotor.get("rightRear"); // Drivebase

        leftSlide = hardwareMap.dcMotor.get("frontEncoder"); // Slides
        rightSlide = hardwareMap.dcMotor.get("Right Slide"); // Slides
        Intake = hardwareMap.dcMotor.get("leftEncoder"); //Pixel Intake

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AHRS imu = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);

        impasta = new Impasta(fl, fr, bl, br, leftSlide, rightSlide, Intake, imu);
        mechanisms = new Mechanisms(gamepad1, gamepad2, hardwareMap);

        schedule(new BulkCacheCommand(hardwareMap));

        // Commands for controlling the lift using buttons
//        mechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> {pidActive = true;}))
//                .whenPressed(new SlidePID(impasta, HIGH).withTimeout(1500))
//                .whenReleased(new InstantCommand(() -> {pidActive = false;}));
//
//        mechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(() -> {pidActive = true;}))
//                .whenPressed(new SlidePID(impasta, MID).withTimeout(1500))
//                .whenReleased(new InstantCommand(() -> {pidActive = false;}));
//
//        mechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new InstantCommand(() -> {pidActive = true;}))
//                .whenPressed(new SlidePID(impasta, LOW).withTimeout(1500))
//                .whenReleased(new InstantCommand(() -> {pidActive = false;}));
//
//        mechanism.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> {pidActive = true;}))
//                .whenPressed(new SlidePID(impasta, GROUND).withTimeout(1500))
//                .whenReleased(new InstantCommand(() -> {pidActive = false;}));

        // Command for resetting the lift
        mechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> impasta.reset()));

        impasta.resetSlide();

        telemetry.addLine("Initialization Done");
        telemetry.update();
    }

    @Override
    public void run(){
        super.run();
        impasta.driveBaseField(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            impasta.reset();
            gamepad1.rumble(250); // Angle recalibrated
        }

        impasta.intake(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad2.left_bumper || gamepad1.right_bumper) {
            impasta.resetSlide();
        }

        if (!pidActive) {
            impasta.runManual(gamepad2.right_stick_y);
            telemetry.addLine("Left Lift Position: " + leftSlide.getCurrentPosition());
        }

        mechanisms.airplaneLauncher();
        mechanisms.v4b();
        mechanisms.outtakes();

        telemetry.update();
    }
}
