package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HorizontalElevator extends SubsystemBase {

    private final CRServo frontIntake;
    private final CRServo backIntake;

    private final Telemetry telemetry;
    private DcMotorSimple.Direction intakeDirection;
    private double intakePower = 0.0;
    public HorizontalElevator(HardwareMap hardwareMap,Telemetry telemetry){
        frontIntake = hardwareMap.get(CRServo.class,"FrontIntake");
        backIntake = hardwareMap.get(CRServo.class,"Back Intake");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        frontIntake.setDirection(intakeDirection);
        frontIntake.setPower(intakePower);

        backIntake.setDirection(intakeDirection);
        backIntake.setPower(intakePower);

//        telemetry.addLine(String.format("Pwr:%4.2f",intakePower));
//        telemetry.addLine(String.format("Dir:%d",intakeDirection == DcMotorSimple.Direction.FORWARD ? 1 : 0));

    }

    public void setIntake(DcMotorSimple.Direction direction,double power){
        intakeDirection = direction;
        intakePower = power;
    }

}
