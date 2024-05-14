package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

import java.util.concurrent.TimeUnit;

public class SlideOuttakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;

    private Timing.Timer timer = new Timing.Timer(1500, TimeUnit.MILLISECONDS);

    private boolean sw = false;

    public SlideOuttakeCommand(IntakeSubsystem intakeSubsystem, SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;

        addRequirements(intakeSubsystem, scoreSubsystem);
    }

    @Override
    public void execute() {
        if(slideSubsystem.getSlideExtensionMeters() >= 0.01) {
            if(!sw) {
                if(!timer.isTimerOn()) {
                    timer.start();
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
                    intakeSubsystem.setIntakePower(-1);
                } else if(timer.elapsedTime() >= 1500) {
                    intakeSubsystem.setIntakePower(0);
                    timer = new Timing.Timer(1500, TimeUnit.MILLISECONDS);
                    sw = true;
                }
            }
        } else {
            sw = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
