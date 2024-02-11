package org.firstinspires.ftc.teamcode.Commands.Procedures;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Subsystems.HorizontalElevator;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public class MoveHorizontalElevator extends CommandBase {

    private VisionSubsystem visionSubsystem;
    private HorizontalElevator horizontalElevator;

    private PIDController controller;

    public MoveHorizontalElevator(double distance,VisionSubsystem visionSubsystem, HorizontalElevator horizontalElevator){
        this.visionSubsystem = visionSubsystem;
        this.horizontalElevator = horizontalElevator;

        controller = new PIDController(0.0,0.0,0.0);
        controller.setSetPoint(distance);

        addRequirements(visionSubsystem,horizontalElevator);
    }

    @Override
    public void execute(){
        horizontalElevator.setExtenderPower(
                controller.calculate(
                        visionSubsystem.getExtenderDistance()
                )
        );
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
