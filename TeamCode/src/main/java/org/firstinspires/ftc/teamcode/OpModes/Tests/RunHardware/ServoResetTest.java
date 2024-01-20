package org.firstinspires.ftc.teamcode.OpModes.Tests.RunHardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="MotorTest ")
@Disabled
public class ServoResetTest extends LinearOpMode {
//
//    private DcMotor motor = null;
        private Servo rightServo;
        private Servo leftServo;

    public ServoResetTest() {
//        if (hardwareMap == null) telemetry.addLine("hardwareMap is null ");
//        hardwareMap.get(DcMotor.class, "Front Left");
    }


    @Override
    public void runOpMode() throws InterruptedException {
//            motor = hardwareMap.get(DcMotor.class,"Back Left");
            rightServo = hardwareMap.get(Servo.class,"right Servo");
            leftServo = hardwareMap.get(Servo.class,"left Servo");
//            telemetry.addLine(String.format("Exists: %b",))
            waitForStart();
            while (true){
                if(gamepad1.a){
                    this.leftServo.setPosition(0.25);
                    this.rightServo.setPosition(0.75);
                }
                if(gamepad1.b){
                    this.leftServo.setPosition(0.75);
                    this.rightServo.setPosition(0.25);
                }
            }


    }
}