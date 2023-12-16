package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.Optional;

public class InformationProcessing extends CommandBase {

    private final VisionSubsystem visionSubsystem;
    private final Telemetry telemetry;

    public InformationProcessing(VisionSubsystem visionSubsystem, Telemetry telemetry){
        this.visionSubsystem = visionSubsystem;
        this.telemetry = telemetry;

        addRequirements(visionSubsystem);
    }

    @Override
    public void execute(){

        Optional<Pose2d> robotPose = visionSubsystem.getRobotPose();

        if(robotPose.isPresent()) {
            telemetry.addLine(String.format("X:%4.2f Y:%4.2f", robotPose.get().getX(), robotPose.get().getY()));
        }else {
            telemetry.addLine("No Position");
        }

        telemetry.update();
    }

}
