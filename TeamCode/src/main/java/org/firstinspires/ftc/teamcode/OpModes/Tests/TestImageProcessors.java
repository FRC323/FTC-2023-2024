package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ExtenderProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Process Images", group = "tests")
public class TestImageProcessors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ExtenderProcessor processor = new ExtenderProcessor(telemetry);

        WebcamName frontCamera = hardwareMap.get(WebcamName.class,"Front Camera");
//        WebcamName backCamera = hardwareMap.get(WebcamName.class,"Back Camera");

        VisionPortal visionPortal;

        visionPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .enableLiveView(true)
                .addProcessor(processor)
                .build();

        visionPortal.resumeLiveView();
        visionPortal.resumeStreaming();

        waitForStart();

        while (!isStopRequested()){
//            visionPortal.resumeStreaming();
            telemetry.addLine(String.format("H: %d",processor.getHeight()));
            telemetry.update();
        }
    }
}
