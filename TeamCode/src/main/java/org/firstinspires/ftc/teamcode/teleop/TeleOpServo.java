package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpServo extends LinearOpMode {
    private Servo servo0;
    private Servo servo1;

    @Override
    public void runOpMode(){
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        servo1.setDirection(Servo.Direction.REVERSE);
        
        servo0.setPosition(0);
        servo1.setPosition(0);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("servo0", servo0.getPosition());
            telemetry.addData("servo1", servo1.getPosition());
            telemetry.update();
        }
    }


}
