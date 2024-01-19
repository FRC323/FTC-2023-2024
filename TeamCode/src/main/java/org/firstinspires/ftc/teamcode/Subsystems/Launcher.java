package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher extends SubsystemBase {

    private final Servo launchServo;

    public Launcher(HardwareMap hardwareMap){
        launchServo = hardwareMap.get(Servo.class,"Airplane");
    }

    public void setLaunched(){
        launchServo.setPosition(0.0);
    }

    public void setBack(){
        launchServo.setPosition(1.0);
    }

}
