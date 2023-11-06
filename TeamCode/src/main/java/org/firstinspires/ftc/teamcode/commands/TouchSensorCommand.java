package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class TouchSensorCommand extends CommandBase {
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public TouchSensorCommand(ScoreSubsystem scoreSubsystem, IntakeSubsystem intakeSubsystem) {
        this.scoreSubsystem = scoreSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(scoreSubsystem);
    }

    @Override
    public void execute() {
        if(scoreSubsystem.isLeftPressed() && scoreSubsystem.isRightPressed()) {
            scoreSubsystem.useClaw(Constants.CLOSE_CLAW_TELEOP);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            intakeSubsystem.setIntakePower(0);
            scoreSubsystem.pivotClaw(Constants.PIVOT_PIVOT_POS);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
        }
    }
}
