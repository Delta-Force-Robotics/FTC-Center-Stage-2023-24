package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

import java.util.function.DoubleSupplier;

public class ClimbManualCommand extends CommandBase {

    private ClimbSubsystem climbSubsystem;
    private DoubleSupplier rightTrigger;
    private DoubleSupplier leftTrigger;

    public ClimbManualCommand(ClimbSubsystem climbSubsystem, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
        this.climbSubsystem = climbSubsystem;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {

            double rightTriggerOutput = rightTrigger.getAsDouble();
            double leftTriggerOutput = leftTrigger.getAsDouble();
            double climbPower = rightTriggerOutput - leftTriggerOutput;

            climbSubsystem.setClimbPower(climbPower);

    }
}
