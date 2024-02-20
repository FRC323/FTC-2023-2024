package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="DriveWheels",group = "tests")
public class MoveDriveWheels extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap hardware_map = hardwareMap;

        //Motors
        Motor frontLeftMotor = new Motor(hardware_map,"Front Left");
        Motor frontRightMotor = new Motor(hardware_map,"Front Right");
        Motor backLeftMotor = new Motor(hardware_map,"Back Left");
        Motor backRightMotor = new Motor(hardware_map,"Back Right/Right Odom");

        frontRightMotor.setInverted(false);
        frontLeftMotor.setInverted(false);
        backLeftMotor.setInverted(false);
        backRightMotor.setInverted(false);

        waitForStart();

        while (!isStopRequested()){
//            frontLeftMotor.set(0.25);
//            frontRightMotor.set(0.25);
            backLeftMotor.set(0.25);
//            backRightMotor.set(0.25);
        }
    }
}
