package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.function.BooleanSupplier;

public class ClimbSubsystem extends SubsystemBase {
    private Motor climbMotor;

    private PIDFController pidfClimbMotor;
    private double calculate;
    private Telemetry telemetry;
    public BooleanSupplier isInterrupted;
    private double position = 0;

/*
    public enum ClimbState {
        INIT(Constants.CLIMB_MOTOR_INIT_POS);

        private double id;
        ClimbState(double climbLevel) {
            id = climbLevel;
        }

        public double getId() { return id; }
        public void setId(double climbLevel) {
            id = climbLevel;
        }
        public boolean isSameClimbLevel(double climbLevel) { return id == climbLevel; }
    }

    public ClimbState climbState = ClimbState.INIT;
*/

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public ClimbSubsystem(Motor climbMotor, Telemetry telemetry, boolean resetEncoder) {
        this.climbMotor = climbMotor;

        this.climbMotor.setRunMode(Motor.RunMode.RawPower);
        this.climbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.climbMotor.setInverted(true);
        this.telemetry = telemetry;

        if (resetEncoder) {
            this.climbMotor.resetEncoder();
        }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*

    /**
     * Sets the extension position for the climbMotor using a PID.
     * @param position Intended position for climb extension, in millimeters.
     */

    /*
    public void setPosition(double position) {
        Constants.CLIMB_INPUT_STATE = Constants.InputState.PRESET_POSITIONS;
        climbState.setId(position);

        try {
            sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        pidfClimbMotor = new PIDFController(1, 0, 0, 0);

        pidfClimbMotor.setSetPoint(position);

        pidfClimbMotor.setTolerance(0.01);

        while(!pidfClimbMotor.atSetPoint() && !isInterrupted.getAsBoolean()) {
            calculate = pidfClimbMotor.calculate(ticksToMeters(climbMotor.getCurrentPosition()));

            climbMotor.set(calculate);

            telemetry.addData("climbMotorMeters", ticksToMeters(climbMotor.getCurrentPosition()));
            telemetry.update();


            try {
                sleep(25);
            } catch (InterruptedException e) {
                Constants.CLIMB_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;

                e.printStackTrace();
            }

        }

        climbMotor.set(0);
        Constants.CLIMB_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;

    }
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void setClimbPower(double power) {
        climbMotor.set(power);
    }
/*
    public double getClimbExtensionMeters() {
        return ticksToMeters(climbMotor.getCurrentPosition());
    }
*/
    public void setClimbPos(int pos) {
        climbMotor.setRunMode(Motor.RunMode.PositionControl);
        climbMotor.setTargetPosition(pos);
        climbMotor.set(0);

        climbMotor.setPositionTolerance(20);

        while (!climbMotor.atTargetPosition() && !isInterrupted.getAsBoolean()) {
            climbMotor.set(0.2);
        }

        climbMotor.stopMotor();
    }
    /*

    public double ticksToMeters(int ticks) {
        return (double) ticks / Constants.CLIMB_MOTOR_MAX_EXTENSION * Constants.CLIMB_MAX_EXTENSION_METERS;
    }

    public double getMotorPos() {
        return climbMotor.getCurrentPosition();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
    public ClimbState getClimbState() {
        return climbState;
    }

    public void setCLimbState(ClimbState climbState) {
        this.climbState = climbState;
    }

    public double metersToTicks(double meters) {
        return Math.round(meters / Constants.CLIMB_MAX_EXTENSION_METERS * Constants.CLIMB_MOTOR_MAX_EXTENSION);
    }

    public int getMotorTicks() {
        return climbMotor.getCurrentPosition();
    }
*/
}
