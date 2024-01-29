package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Impasta {
    // Hardware variables
    private AHRS imu;
    private IMU.Parameters parameters;
    private DcMotor fl, fr, bl, br, leftSlide, rightSlide, Intake;
    private Servo leftOut, rightOut, leftV4B, rightV4B;
    private PID pid = new PID(0.02, 0, 0);

    private boolean leftOutRaised = true;
    private boolean rightOutRaised = true;

    private boolean v4bAtRest = true;


    // Constructor for Impasta class
    public Impasta(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor leftSlide, DcMotor rightSlide, DcMotor Intake, AHRS imu) {
        // Assigning hardware references to local variables
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.Intake = Intake;
        this.imu = imu;

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Impasta(DcMotor leftSlide, DcMotor rightSlide) {
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
    }

    public Impasta(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor leftSlide, DcMotor rightSlide, DcMotor Intake) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.Intake = Intake;

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
    }

    public Impasta(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor leftSlide, DcMotor rightSlide, DcMotor Intake, Servo leftOut, Servo rightOut, Servo leftV4B, Servo rightV4b) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.leftSlide = leftSlide;
        this.rightSlide = rightSlide;
        this.Intake = Intake;
        this.leftOut = leftOut;
        this.rightOut = rightOut;
        this.leftV4B = leftV4B;
        this.rightV4B = rightV4b;

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
    }

    // Method to reset IMU yaw angle
    public void reset() {
        imu.zeroYaw();
    }

    // Method to drive the robot based on gamepad input
    public void driveBaseRobot(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double flpwr = (y + x + rx) / denominator;
        double blpwr = (y - x + rx) / denominator;
        double frpwr = (y - x - rx) / denominator;
        double brpwr = (y + x - rx) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }

    public void driveBaseField(double y, double x, double rx) {
        double botHeading = Math.toRadians(imu.getYaw());

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double flpwr = (rotY + rotX + rx) / denominator;
        double blpwr = (rotY - rotX + rx) / denominator;
        double frpwr = (rotY - rotX - rx) / denominator;
        double brpwr = (rotY + rotX - rx) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }

    // Method to control intake motor
    public void intake(double ip) {
        Intake.setPower(ip);
    }

    //Slide Control via power


//  ================================================ Slide Stuff ================================================

    public void setSlidesPower(double power) {
        leftSlide.setPower(-power);
        rightSlide.setPower(power);
    }

    public void runManual(double dr4bp) {
        leftSlide.setPower(-dr4bp);
        rightSlide.setPower(dr4bp);
    }

    public double getSlidesPos() {
        return leftSlide.getCurrentPosition();
    }

    public void runPID(double targetPos) {
        double output = pid.update(leftSlide.getCurrentPosition(), targetPos);
        leftSlide.setPower(output);
        rightSlide.setPower(-output);
    }

    public void resetSlide() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean atUpper() {
        return getSlidesPos() > 600;
    }

    public boolean atLower() {
        return getSlidesPos() < 5;
    }

//  =============================================================================================================

    public void toggleLeftOut() {
        if (leftOutRaised) {
            leftOut.setPosition(0.75);
        } else {
            leftOut.setPosition(0.55);
        }
        leftOutRaised = !leftOutRaised;
    }

    public void toggleRightOut() {
        if (rightOutRaised) {
            rightOut.setPosition(0.5);
        } else {
            rightOut.setPosition(0.6);
        }
        rightOutRaised = !rightOutRaised;
    }

    public void toggleV4B() {
        if (v4bAtRest) {
            leftV4B.setPosition(180);
            rightV4B.setPosition(180);
        } else {
            leftV4B.setPosition(0);
            rightV4B.setPosition(0);
        }
        v4bAtRest = !v4bAtRest;
    }

    public double getLeftV4Bpos() {
        return leftV4B.getPosition();
    }

    public double getRightV4Bpos() {
        return rightV4B.getPosition();
    }

    public double getLeftOutPos() {
        return leftOut.getPosition();
    }

    public double getRightOutPos() {
        return rightOut.getPosition();
    }

    public void driveBaseAuto(double x) {
        double denominator = Math.max(Math.abs(0) + Math.abs(x) + Math.abs(0), 1);

        double flpwr = (0 + x + 0) / denominator;
        double blpwr = (0 - x + 0) / denominator;
        double frpwr = (0 - x - 0) / denominator;
        double brpwr = (0 + x - 0) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }

    public void driveBaseAutoFar(double y) {
        double denominator = Math.max(Math.abs(y) + Math.abs(0) + Math.abs(0), 1);

        double flpwr = (y + 0 + 0) / denominator;
        double blpwr = (y - 0 + 0) / denominator;
        double frpwr = (y - 0 - 0) / denominator;
        double brpwr = (y + 0 - 0) / denominator;

        fl.setPower(flpwr);
        bl.setPower(blpwr);
        fr.setPower(frpwr);
        br.setPower(brpwr);
    }
}
