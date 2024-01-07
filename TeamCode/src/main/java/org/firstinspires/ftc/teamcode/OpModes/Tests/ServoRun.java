package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Run",group = "tests")
public class ServoRun extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo_0 = hardwareMap.get(CRServo.class,"S0");
        CRServo servo_1 = hardwareMap.get(CRServo.class,"S1");
        CRServo servo_2 = hardwareMap.get(CRServo.class,"S2");
        CRServo servo_3 = hardwareMap.get(CRServo.class,"S3");
        CRServo servo_4 = hardwareMap.get(CRServo.class,"S4");
        CRServo servo_5 = hardwareMap.get(CRServo.class,"S5");

        waitForStart();

        while (!isStopRequested()){
            servo_0.setPower(1.0);
            servo_1.setPower(1.0);
            servo_2.setPower(1.0);
            servo_3.setPower(1.0);
            servo_4.setPower(1.0);
            servo_5.setPower(1.0);
        }
    }
}
