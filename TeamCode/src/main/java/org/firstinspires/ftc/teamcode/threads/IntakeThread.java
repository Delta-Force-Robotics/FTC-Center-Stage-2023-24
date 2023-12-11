package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeThread extends Thread {
    private IntakeSubsystem intakeSubsystem;
    public double levelForSlides = 0;
    boolean isAuto;

    public IntakeThread(IntakeSubsystem intakeSubsystem, boolean isAuto) {
        this.intakeSubsystem = intakeSubsystem;
        this.isAuto = isAuto;
    }

    @Override
    public void run() {
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
        intakeSubsystem.setIntakePower(0.6);
    }
}
