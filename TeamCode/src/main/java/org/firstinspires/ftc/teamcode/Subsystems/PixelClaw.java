package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class PixelClaw extends SubsystemBase {
    private final Servo leftServo;
    private  final Servo rightServo;

    private final ColorSensor colorSensor;


    private final double CLOSED_POSITION_LEFT = 0.0;
    private final double CLOSED_POSITION_RIGHT = 1.0;
    private final double OPEN_POSITION_LEFT = 1.0;
    private final double OPEN_POSITION_RIGHT = 0.0;


    private final int PURPLE_PIXEL_COLOR = 0xFF00FF;
    private final int GREEN_PIXEL_COLOR = 0x00FF00;
    private final int YELLOW_PIXEL_COLOR = 0xFFFF00;
    private final int WHITE_PIXEL_COLOR = 0xFFFFFF;

    private final long DROP_TIME = 1;

    Timing.Timer dropBuffer = new Timing.Timer(1, TimeUnit.SECONDS);

    private boolean previouslyClosed;
    enum PixelColor{
        Purple,
        Yellow,
        Green,
        White
    }

    private final double NO_PIXEL_RED = 0x00;

    public PixelClaw(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "Left Claw");
        rightServo = hardwareMap.get(Servo.class, "Right Claw");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
    }

    @Override
    public void periodic(){
        if(hasPixel() && dropBuffer.done()){
            setClosed();
        }
    }

    public void setClosed(){
        leftServo.setPosition(CLOSED_POSITION_LEFT);
        rightServo.setPosition(CLOSED_POSITION_RIGHT);
        previouslyClosed = true;
    }

    public void setOpen(){
        leftServo.setPosition(OPEN_POSITION_LEFT);
        rightServo.setPosition(OPEN_POSITION_RIGHT);
        resetTimer();
    }

    public boolean hasPixel(){
        //TODO: Get Actual value
        return colorSensor.red() > NO_PIXEL_RED;
    }

    public PixelColor getPixelColor(){
        //TODO
        return PixelColor.White;
    }

    private void resetTimer(){
        dropBuffer.pause();
        dropBuffer = new Timing.Timer(DROP_TIME,TimeUnit.SECONDS);
        dropBuffer.start();
    }

}
