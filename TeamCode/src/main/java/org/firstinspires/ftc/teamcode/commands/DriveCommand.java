package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
    * This is our drive command. This works hand in hand with the DriveSubsystem class.
    * Using our DualShock 4 Controllers we control the chassis.
    * The motors get assigned power from -1 to 1 based on how far we push the joysticks.
     **/

public class DriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotation;

    /**
     * @param driveSubsystem    The drive subsystem this command wil run on.
     * @param forward           The control input for driving forwards/backward
     * @param rotation          The control input for turning
     **/

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward, DoubleSupplier rotation) {
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.rotation = rotation;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(forward.getAsDouble(), rotation.getAsDouble());
    }
}
