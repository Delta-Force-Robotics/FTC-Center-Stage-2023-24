package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

import java.util.function.DoubleSupplier;

public class SlideManualCommand extends CommandBase {

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private DoubleSupplier rightTrigger;
    private DoubleSupplier leftTrigger;

    public SlideManualCommand(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;

        addRequirements(slideSubsystem);
    }

    @Override
    public void execute() {
        if(Constants.SLIDE_INPUT_STATE == Constants.InputState.MANUAL_CONTROL) {
            double rightTriggerOutput = rightTrigger.getAsDouble();
            double leftTriggerOutput = leftTrigger.getAsDouble();
            double slidePower = leftTriggerOutput - rightTriggerOutput;

            slideSubsystem.setMotorPower(slidePower);
            //telemetry.addData("slidePower", slidePower);
            //telemetry.update();
        }

        if (slideSubsystem.getMotorTicks() <= 30) {
            scoreSubsystem.isScoring = true;
        }
    }
}
