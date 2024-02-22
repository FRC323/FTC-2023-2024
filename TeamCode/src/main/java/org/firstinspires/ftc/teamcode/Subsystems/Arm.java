package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    private Motor leftMotor;
    private Motor rightMotor;
    private Motor.Encoder armEncoder;

    //Controllers
    private PIDController armController = new PIDController(
            1.5,
            0.0,
            0.0
    );

    private ArmFeedforward armFeedforward = new ArmFeedforward(
           0.0,
           0.0,
           0.0
    );

    private double ENCODER_OFFSET = 10;
    private double TICKS_PER_RADS = 3226 / Math.PI;

    private Telemetry telemetry;

    public final double HIGH_SCORE = 3.0;
    public final double LOW_SCORE = 4.0;
    public final double CLIMB_UP = 2.25;
    public final double INTAKE_DOWN = 0.1;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry){
        leftMotor = new Motor(hardwareMap,"Left Arm");
        rightMotor = new Motor(hardwareMap,"Right Arm");
        armEncoder = leftMotor.encoder;

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic(){
        double power = armController.calculate(armEncoder.getPosition() / TICKS_PER_RADS)
                + armFeedforward.calculate((armEncoder.getPosition() - 739) / TICKS_PER_RADS,armEncoder.getCorrectedVelocity());

        leftMotor.set(power);
        rightMotor.set(-power);

        telemetry.addLine(String.format("Arm Encoder: %d",armEncoder.getPosition()));
        telemetry.update();
    }

    public void setTargetRads(double position){
        armController.setSetPoint(position);
    }

    public double getAngleRads(){
        return armEncoder.getPosition() / TICKS_PER_RADS;
    }

    public void setArmMotorPower(double power){
        leftMotor.set(power);
        rightMotor.set(-power);
    }

}
