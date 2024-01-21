package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;

public class ClimbThread extends Thread{
    public ClimbSubsystem climbSubsystem;

    public double position = 0;

    public ClimbThread (ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.climbSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        Constants.CLIMB_INPUT_STATE = Constants.InputState.PRESET_POSITIONS;
        //climbSubsystem.setPosition(position);
        climbSubsystem.setClimbPos((int) position);
    }

    public void interrupt() {
        Constants.CLIMB_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
