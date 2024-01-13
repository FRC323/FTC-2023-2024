package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name = "ColorSensor",group = "tests")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        colorSensor.enableLed(false);
        colorSensor.setI2cAddress(I2cAddr.create8bit(1));

        waitForStart();

        while (!isStopRequested()){
            telemetry.addLine(String.format("Red: %d",colorSensor.red()));
            telemetry.addLine(String.format("Green: %d",colorSensor.green()));
            telemetry.addLine(String.format("Blue: %d",colorSensor.blue()));

        }
    }


}
