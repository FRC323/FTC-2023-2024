package org.firstinspires.ftc.teamcode.OpModes.Tests.RunHardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Wheels",group = "tests")
@Disabled
public class MotorFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor0 = hardwareMap.get(DcMotor.class,"Back Right/Arm Encoder");
        DcMotor motor1 = hardwareMap.get(DcMotor.class,"Front Right/Center Odom");
        DcMotor motor2 = hardwareMap.get(DcMotor.class,"Front Left/Left Odom");
        DcMotor motor3 = hardwareMap.get(DcMotor.class,"Back Left");

        waitForStart();

        while (!isStopRequested()){
            if(gamepad1.a) motor0.setPower(0.5);
            else motor0.setPower(0.0);

            if(gamepad1.b) motor1.setPower(0.5);
            else motor1.setPower(0.0);

            if(gamepad1.x) motor2.setPower(0.5);
            else motor2.setPower(0.0);

            if(gamepad1.y) motor3.setPower(0.5);
            else motor3.setPower(0.0);

            telemetry.addLine(String.format("Pwr: ",motor0.getController().getMotorPower(0)));
            telemetry.update();
        }
    }
}
