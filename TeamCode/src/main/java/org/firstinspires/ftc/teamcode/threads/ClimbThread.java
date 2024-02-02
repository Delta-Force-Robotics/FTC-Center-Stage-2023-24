package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;

public class ClimbThread extends Thread{

    ClimbSubsystem climbSubsystem;
    public double climbPos = 0;

    public ClimbThread (ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.climbSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        climbSubsystem.setClimbPos((int)climbPos);
    }

    public void interrupt() {
        Constants.CLIMB_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
