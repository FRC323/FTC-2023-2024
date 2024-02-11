package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ExtenderPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "Image",group = "tests")
@Disabled
public class ImageProc extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Front Camera");

        telemetry.addLine("Attached? " + webcamName.isAttached());

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        ExtenderPipeline pipeline = new ExtenderPipeline();


        waitForStart();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                telemetry.addLine("Camera Working");
                telemetry.update();
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(pipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("ErrorCode: " + errorCode);
                telemetry.update();
            }
        });


        while (!isStopRequested()){
            if(gamepad1.x) pipeline.setImageToReturn(0);
            if(gamepad1.a) pipeline.setImageToReturn(1);
            if(gamepad1.b) pipeline.setImageToReturn(2);
            if(gamepad1.y) pipeline.setImageToReturn(3);

            telemetry.addLine("C: " + pipeline.getDistanceFromFrame());
            telemetry.update();
        }
    }
}
