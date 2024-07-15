package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class PivotCommand extends CommandBase {
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public PivotCommand(ScoreSubsystem scoreSubsystem, SlideSubsystem slideSubsystem, IntakeSubsystem intakeSubsystem) {
        this.scoreSubsystem = scoreSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.slideSubsystem = slideSubsystem;

        addRequirements(scoreSubsystem);
    }

    @Override
    public void execute() {
        if (slideSubsystem.getMotorTicks() > 450 && scoreSubsystem.isScoring) {
            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_PIVOT_POSITION);
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
            scoreSubsystem.isScoring = false;
        }
    }
}
