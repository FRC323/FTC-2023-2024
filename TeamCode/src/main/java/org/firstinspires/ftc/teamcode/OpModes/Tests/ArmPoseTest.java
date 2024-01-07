package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Arm",group = "tests")
public class ArmPoseTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo Arm1 = hardwareMap.get(Servo.class,"Arm 1");
        Servo Arm2 = hardwareMap.get(Servo.class,"Arm 2");

        waitForStart();

        while (!isStopRequested()){
            if(gamepad2.a){
                Arm1.setPosition(0.01);
                Arm2.setPosition(0.99);
            }

            if(gamepad2.b){
                Arm1.setPosition(0.5);
                Arm2.setPosition(0.5);
            }

            if(gamepad2.x){
                Arm1.setPosition(1.0);
                Arm2.setPosition(0.0);
            }

            if(gamepad2.y){
                Arm1.setPosition(0.25);
                Arm2.setPosition(0.75);
            }
        }
    }
}
