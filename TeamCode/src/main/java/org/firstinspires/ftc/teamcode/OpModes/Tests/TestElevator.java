package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Elevator")
@Disabled
public class TestElevator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor elevatorMotor = hardwareMap.get(DcMotor.class,"Elevator Motor");
        GamepadEx gamepad = new GamepadEx(gamepad2);


        waitForStart();

        while (true) {
//            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine(String.format("%4.2f",gamepad.getRightY()));
            telemetry.update();
              elevatorMotor.setPower(gamepad.getRightY());

        }

    }
}
