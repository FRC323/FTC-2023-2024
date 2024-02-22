package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(name = " Servo Run")
public class servoRun extends LinearOpMode {
    private Servo left;
    private Servo right;
    private Servo airplane;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo Servo0 = hardwareMap.get(Servo.class,"0");
        Servo Servo1 = hardwareMap.get(Servo.class,"1");
        Servo Servo2 = hardwareMap.get(Servo.class,"2");
        Servo Servo3 = hardwareMap.get(Servo.class,"3");
        Servo Servo4 = hardwareMap.get(Servo.class,"4");
        Servo Servo5 = hardwareMap.get(Servo.class,"5");

        Servo Exp0 = hardwareMap.get(Servo.class,"E0");
        Servo Exp1 = hardwareMap.get(Servo.class,"E1");
        Servo Exp2 = hardwareMap.get(Servo.class,"E2");
        Servo Exp3 = hardwareMap.get(Servo.class,"E3");
        Servo Exp4 = hardwareMap.get(Servo.class,"E4");
        Servo Exp5 = hardwareMap.get(Servo.class,"E5");


        waitForStart();


        while(!isStopRequested()){
            if(gamepad1.a){
                Servo0.setPosition(1.0);
                Servo1.setPosition(1.0);
                Servo2.setPosition(1.0);
                Servo3.setPosition(1.0);
                Servo4.setPosition(1.0);
                Servo5.setPosition(1.0);

                Exp0.setPosition(1.0);
                Exp1.setPosition(1.0);
                Exp2.setPosition(1.0);
                Exp3.setPosition(1.0);
                Exp4.setPosition(1.0);
                Exp5.setPosition(1.0);
            }

            if(gamepad1.a){
                Servo0.setPosition(0.0);
                Servo1.setPosition(0.0);
                Servo2.setPosition(0.0);
                Servo3.setPosition(0.0);
                Servo4.setPosition(0.0);
                Servo5.setPosition(0.0);

                Exp0.setPosition(0.0);
                Exp1.setPosition(0.0);
                Exp2.setPosition(0.0);
                Exp3.setPosition(0.0);
                Exp4.setPosition(0.0);
                Exp5.setPosition(0.0);
            }
        }
    }
}
