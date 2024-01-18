package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;


public class BackdropProcessor implements VisionProcessor {

    private TfodProcessor tfodProcessor;

    private List<List<Recognition>> backdropState;

    private final double MaxHeightError = 0.5;

    public static enum PixelColor{
        White,
        Green,
        Purple,
        Yellow,
        None
    }

    BackdropProcessor(){
        tfodProcessor = new TfodProcessor.Builder()
                //TODO:Get stuff for here
                .setModelLabels(new String[]{
                        "White",
                        "Green",
                        "Purple",
                        "Yellow"
                })
                .build();


    }


    public void updateBackdropState(){
        backdropState = new ArrayList<>();

        List<Recognition> recognitions = tfodProcessor.getRecognitions();
        recognitions.sort(((a, b) -> {
            //Compares the heights of the items in the list to sort them
            return (a.getTop() - b.getTop()) > 0 ? 1 : -1;
        }));

        int level = 1;
        int startOfLevel = 0;

        while(startOfLevel < recognitions.size()){
            List<Recognition> row = new ArrayList<>();

            Recognition startPixel = recognitions.get(startOfLevel);

            //gets all pixels that belong to the same level as the startOfLevel Pixel
            row = recognitions.stream().filter((recognition) ->{
                return
                        //Checks if Pixel is too High
                        (recognition.getTop() - (MaxHeightError*startPixel.getHeight())) < startPixel.getTop()
                        //Checks if Pixel is too Low
                        && (recognition.getTop() + (MaxHeightError*startPixel.getHeight())) > startPixel.getTop();
            }).collect(Collectors.toList());

            //Sorts Row from left to right
            row.sort((a,b) -> {
                return (a.getRight() - b.getRight()) > 0 ? 1 : -1;
            });

            //Moves index of
            startOfLevel += row.size();

            backdropState.add(row);
        }

        //TODO: REMEBER Recognition.getLabel() exists for pixel color
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
