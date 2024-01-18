package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;
import java.util.Optional;

public class TeamPropProcessor implements VisionProcessor {

    private TfodProcessor tfodProcessor;

    private float MIN_CONFIDENCE = (float) 0.0;

    public static enum PropLocation{
        Left,Center,Right
    }

    //TODO: Get actual values of postions
    private float LEFT_POSITION = (float) 20;
    private float RIGHT_POSITION = (float) 20;



    TeamPropProcessor(){
        tfodProcessor = new TfodProcessor.Builder()
                //TODO
                .setModelLabels(new String[]{
                        "RedAlliance",
                        "BlueAlliance"
                })
                .build();
    }





    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        tfodProcessor.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return tfodProcessor.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        tfodProcessor.onDrawFrame(canvas, onscreenWidth,onscreenHeight,scaleBmpPxToCanvasPx,scaleCanvasDensity, userContext);
    }
}
