package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import org.checkerframework.checker.nullness.qual.Raw;

public class DriveSubsystem extends SubsystemBase {
    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;

    private MotorGroup motorGroupLeft;
    private MotorGroup motorGroupRight;

    private DifferentialDrive sixWheelDrive;

    public DriveSubsystem(Motor leftFront, Motor leftBack, Motor rightFront, Motor rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;

        motorGroupLeft = new MotorGroup(leftFront, leftBack);
        motorGroupRight = new MotorGroup(rightFront, rightBack);

        motorGroupLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorGroupRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorGroupLeft.setRunMode(Motor.RunMode.RawPower);
        motorGroupRight.setRunMode(Motor.RunMode.RawPower);

        motorGroupRight.setInverted(true);

        sixWheelDrive = new DifferentialDrive(true, motorGroupLeft, motorGroupRight);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation movement
     */
    public void drive(double fwd, double rot) {
        fwd = (Math.abs(fwd) >= 0.1) ? fwd : 0;
        rot = (Math.abs(rot) >= 0.1) ? rot : 0;

        sixWheelDrive.arcadeDrive(fwd, rot, true);
    }
}
