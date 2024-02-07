package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SlidePID extends CommandBase {
    private Impasta impasta;
    private double target;



    public SlidePID(Impasta slides, double target) {
        this.impasta = slides;
        this.target = target;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        impasta.runPID(target);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(impasta.getSlidesPos() - target) < 25;
    }

    @Override
    public void end(boolean interupted) {
        impasta.runManual(0, false);
    }
}
