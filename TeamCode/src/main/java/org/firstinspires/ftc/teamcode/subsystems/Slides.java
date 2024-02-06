package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides extends SubsystemBase {

    public enum SlidePos {
        DOWN(50),
        LOW(70),
        MED(90),
        HIGH(130);

        public int position;

        SlidePos(int position) {
            this.position = position;
        }
    }

    private PID pid = new PID(0.02, 0, 0);

    private DcMotorEx lMotor, rMotor;

    public Slides(HardwareMap hardwareMap) {
        lMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        rMotor = hardwareMap.get(DcMotorEx.class, "Right Slide");

        lMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {}

    private void setPower(double power) {
        lMotor.setPower(-power);
        rMotor.setPower(power);
    }

    public void setSlidesPower(double power) {
        lMotor.setPower(-power);
        rMotor.setPower(power);
    }

    public void runManual(double dr4bp) {
        lMotor.setPower(-dr4bp);
        rMotor.setPower(dr4bp);
    }

    public void stop() {
        setPower(0.1);
    }

    public double getPos() {
        return lMotor.getCurrentPosition();
    }

    public void goToPosition(double targetPos) {
        double output = pid.update(lMotor.getCurrentPosition(), targetPos);
        lMotor.setPower(output);
        rMotor.setPower(-output);
    }
    public boolean atUpper() {
        return getPos() > 600;
    }

    public boolean atLower() {
        return getPos() < 5;
    }

    public void resetSlide() {
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
