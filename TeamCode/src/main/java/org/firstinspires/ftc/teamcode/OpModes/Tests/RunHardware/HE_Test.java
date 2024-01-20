package org.firstinspires.ftc.teamcode.OpModes.Tests.RunHardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HE_TEST",group = "tests")
@Disabled
public class HE_Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rearMotor = hardwareMap.get(DcMotor.class,"extender 1");
        DcMotor frontMotor = hardwareMap.get(DcMotor.class,"extender 2");

        Servo poseServo = hardwareMap.get(Servo.class,"Intake Pose");

        waitForStart();

        while (true){

            rearMotor.setPower(gamepad1.left_stick_x);
            frontMotor.setPower(gamepad1.left_stick_x);

            poseServo.setPosition((gamepad1.right_stick_x+1.0)/2);

            telemetry.addLine(String.format("Pose;%4.2f",poseServo.getPosition()));
            telemetry.update();
            if(isStopRequested()) break;
        }
    }
}
