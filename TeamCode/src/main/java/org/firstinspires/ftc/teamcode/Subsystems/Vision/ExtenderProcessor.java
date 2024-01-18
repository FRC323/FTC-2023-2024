package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class ExtenderProcessor implements VisionProcessor {
    public ExtenderProcessor(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    private Telemetry telemetry;
    //Constants
    private final int START_COL = 0;
    private final int END_COL = 25;

    private final int MIN_HEIGHT = 25;
    private final int MAX_HEIGHT = 50;

    private final int MIN_WIDTH = 25;
    private final int MAX_WIDTH = 50;

    private final Scalar MIN_COLOR = new Scalar(36,145,120);
    private final Scalar MAX_COLOR = new Scalar(175,243,255);

    private final Scalar CONTOUR_COLOR = new Scalar(255,0,0);

    //Variables
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchey = new Mat();
    private Mat lastFrame = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        contours = findExtenderContour(frame);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public int getHeight(){
        if(contours.isEmpty()) return -1;

        return contours.get(0).height();
    }

    private List<MatOfPoint> findExtenderContour(Mat frame){
        Mat frameAdjusted = new Mat();
        Imgproc.cvtColor(frame,frameAdjusted,Imgproc.COLOR_RGB2BGR);

        //Hides all color outside of the specified range
//        Mat colorMasked = new Mat();
//        Core.inRange(frame,MIN_COLOR,MAX_COLOR,colorMasked);

        //finds the bounding box around the color
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
//                colorMasked,
                frame,
                contours,
                hierarchey,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        //returns the contour with the greatest height
//        Optional<MatOfPoint> contour = contours.stream().reduce(
//                (a,c) ->{
//                    return c.height() > a.height() ?
//                            c : a;
//                }
//        );

//        contours.stream().filter(
//                (contour) -> {
//                    return
//                            contour.height() > MIN_HEIGHT &&
//                            contour.height() < MAX_HEIGHT &&
//                            contour.width() > MIN_WIDTH &&
//                            contour.width() < MAX_WIDTH;
//                }
//        );

        return contours;

    }


}
