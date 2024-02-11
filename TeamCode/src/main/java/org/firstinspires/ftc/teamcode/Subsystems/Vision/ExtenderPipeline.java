package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static java.lang.Math.abs;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ExtenderPipeline extends OpenCvPipeline {

//    private final Scalar MAX_COLOR = new Scalar(255,225,225); //(56, 96, 117)
//    private final Scalar MIN_COLOR = new Scalar(180,155,95); //181,247,254
    private final Scalar MAX_COLOR = new Scalar(255,255,200);
    private final Scalar MIN_COLOR = new Scalar(245,225,70);

    //Mats
    private Mat color = new Mat();
    private Mat hsv = new Mat();
    private Mat rectangleMat = new Mat();

    private Rect rectangle = new Rect();

    private static final double[][] extenderDistanceLookup = {
            {360.0,65.0},
            {363.0,55.0},
            {370.0,46.0},
            {380.0,37.0},
            {392.0,28.0},
            {410.0,19.0},
            {428.0,11.0},
            {449.0,6.0},
            {456.0,5.0},
            {463.0,4.0},
            {474.0,0.0},
            //Note: all of these are offset 7.5 inches from the magnetic sensor

    };

    private int phaseToReturn = -1;

    @Override
    public Mat processFrame(Mat input) {
        color = isolateColor(input);
        rectangle = getRectangle(color);
        rectangleMat = getDrawnRectangle(rectangle,input);

        switch(phaseToReturn){
            case 0:
                return input;
            case 1:
                return color;
            case 2:
                return  rectangleMat;
            default:
                return input;
        }
    }

    public double getDistanceFromFrame(){
        if(rectangle == null) return -1.0;

        if(rectangle.y == 0) return 0.0;

        double[] prevKey = {0,0};
        for(double[] key : extenderDistanceLookup){
            if(key[0] > rectangle.y){
                double percent = (key[0] - rectangle.y) / (key[0] - prevKey[1]);
                return key[1] + (abs(key[1] - prevKey[1]) * percent);
            }
            prevKey = key;
        }

        return extenderDistanceLookup[extenderDistanceLookup.length-1][0];
    }

    public int getRawPixelsFromFrame(){
        if(rectangle == null) return -1;
        return rectangle.y;
    }

    public Mat getDrawnRectangle(Rect rect,Mat input){
        rectangleMat = input;
        Imgproc.rectangle(rectangleMat,rect,new Scalar(0,100,255),8);
        return rectangleMat;
    }

    public Mat isolateColor(Mat input){
        Imgproc.cvtColor(input,hsv,Imgproc.COLOR_BGR2RGB);
        Core.inRange(hsv,MIN_COLOR,MAX_COLOR,color);
        return color;
    }

    public Rect getRectangle(Mat input){
        return Imgproc.boundingRect(input);
    }

    public void setImageToReturn(int phase){
        phaseToReturn = phase;
    }


//    public Mat convertToGray(Mat input){
//        Imgproc.cvtColor(input,gray, Imgproc.COLOR_BGR2GRAY);
//        return gray;
//    }
//
//    public Mat convertToBinary(Mat input){
//        //Converts Gray to Binary
//        Imgproc.threshold(input,binary,250,255,0);
//        return binary;
//    }


}
