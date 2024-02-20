package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

import kotlin.Pair;

public class PixelClaw extends SubsystemBase {
    private final Servo leftServo;
    private  final Servo rightServo;

//    private final ColorSensor leftColorSensor;
//    private final  ColorSensor rightColorSensor;


    private final double CLOSED_POSITION_LEFT = 0.0;
    private final double CLOSED_POSITION_RIGHT = 1.0;
    private final double OPEN_POSITION_LEFT = 1.0;
    private final double OPEN_POSITION_RIGHT = 0.0;


    private final Pair<Integer,Integer> PURPLE_PIXEL_COLOR = new Pair(0,0);
    private final Pair<Integer,Integer> GREEN_PIXEL_COLOR = new Pair(0,0);
    private final Pair<Integer,Integer> YELLOW_PIXEL_COLOR = new Pair(0,0);
    private final Pair<Integer,Integer> WHITE_PIXEL_COLOR = new Pair(0,0);
    private final long DROP_TIME = 1;

    Timing.Timer dropBuffer = new Timing.Timer(1, TimeUnit.SECONDS);

    private boolean previouslyClosed;
    enum PixelColor{
        Purple,
        Yellow,
        Green,
        White,
        None
    }

    private final double NO_PIXEL_RED = 0x00;

    public PixelClaw(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "Left Claw");
        rightServo = hardwareMap.get(Servo.class, "Right Claw");
//        leftColorSensor = hardwareMap.get(ColorSensor.class, "Left Color");
//        rightColorSensor = hardwareMap.get(ColorSensor.class, "Right Color");
    }

    @Override
    public void periodic(){
        if(leftHasPixel() && dropBuffer.done()){
            setLeftClosed();
        }
    }

    public void setLeftClosed(){
        leftServo.setPosition(CLOSED_POSITION_LEFT);
        rightServo.setPosition(CLOSED_POSITION_RIGHT);
        previouslyClosed = true;
    }

    public void setLeftOpen(){
        leftServo.setPosition(OPEN_POSITION_LEFT);
        rightServo.setPosition(OPEN_POSITION_RIGHT);
        resetTimer();
    }

    public void setRightClosed(){
        rightServo.setPosition(CLOSED_POSITION_RIGHT);
        previouslyClosed = true;
    }

    public void setRightOpen() {
        rightServo.setPosition(OPEN_POSITION_RIGHT);
        resetTimer();
    }

    public boolean leftHasPixel(){
        return leftPixelColor() != PixelColor.None;
    }

    public  boolean rightHasPixel(){
        return rightPixelColor() != PixelColor.None;
    }

    public PixelColor leftPixelColor(){
        return PixelColor.White;//leftColorSensor.argb();
    }

    public PixelColor rightPixelColor(){
        return PixelColor.White;//rightColorSensor.argb();
    }

    private void resetTimer(){
        dropBuffer.pause();
        dropBuffer = new Timing.Timer(DROP_TIME,TimeUnit.SECONDS);
        dropBuffer.start();
    }

}
