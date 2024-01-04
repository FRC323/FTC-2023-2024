package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HorizontalElevator extends SubsystemBase {
    //Hardware
    private final CRServo frontIntake;
    private final CRServo backIntake;

    private final Motor rearExtenderMotor;
    private final Motor frontExtendorMotor;

    private final Servo intakePoseServo;

    private final Telemetry telemetry;
    private DcMotorSimple.Direction frontIntakeDirection;
    private DcMotorSimple.Direction rearIntakeDirection;

    private double frontIntakePower = 0.0;
    private double rearIntakePower = 0.0;

    public enum IntakePose{
        GROUND,
        HORIZONTAL,
        VERTICAL,
        STARTING
    }

    private double extenderPower = 0.0;
    public HorizontalElevator(HardwareMap hardwareMap,Telemetry telemetry){
        frontIntake = hardwareMap.get(CRServo.class,"Front Intake");
        backIntake = hardwareMap.get(CRServo.class,"Back Intake");

        rearExtenderMotor = new Motor(hardwareMap,"Back Extender");
        frontExtendorMotor = new Motor(hardwareMap,"Front Extender/Right Odom");

        rearExtenderMotor.setRunMode(Motor.RunMode.RawPower);
        frontExtendorMotor.setRunMode(Motor.RunMode.RawPower);

        intakePoseServo = hardwareMap.get(Servo.class,"Intake Pose");

        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        frontIntake.setDirection(frontIntakeDirection);
        frontIntake.setPower(frontIntakePower);

        backIntake.setDirection(rearIntakeDirection);
        backIntake.setPower(rearIntakePower);

        rearExtenderMotor.set(extenderPower);
        frontExtendorMotor.set(extenderPower);

//        telemetry.addLine(String.format("Pwr:%4.2f",intakePower));
//        telemetry.addLine(String.format("Dir:%d",intakeDirection == DcMotorSimple.Direction.FORWARD ? 1 : 0));

        telemetry.addLine(String.format("Pos;%4.2f",intakePoseServo.getPosition()));
    }

    public void setFrontIntake(DcMotorSimple.Direction direction,double power){
        frontIntakeDirection = direction;
        frontIntakePower = power;
    }

    public void setRearIntake(DcMotorSimple.Direction direction, double power){
        rearIntakeDirection = direction;
        rearIntakePower = power;
    }

    public void setExtenderPower(double power){
        extenderPower = -power;
    }

    public void setIntakePosition(IntakePose pose){
        double position = 0.0;
        switch (pose){
            case GROUND:
                position = 0.50;
                break;
            case HORIZONTAL:
                position = 0.60;
                break;
            case VERTICAL:
                position = 0.90;
                break;
            case STARTING:
                position = 1.0;
        }
        intakePoseServo.setPosition(position);
    }

}
