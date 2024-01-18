package org.firstinspires.ftc.teamcode.Commands.Procedures;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.HorizontalElevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.BackdropProcessor.PixelColor;

public class HandoffPixel extends CommandBase {

    private final ScoringMechanism scoringMechanism;
    private final HorizontalElevator horizontalElevator;

    private long colorTime;

    HandoffPixel(ScoringMechanism scoringMechanism, HorizontalElevator horizontalElevator){
        this.scoringMechanism = scoringMechanism;
        this.horizontalElevator = horizontalElevator;
    }

    @Override
    public void execute(){

        //Send the Arm and extender to the proper positions
        scoringMechanism.setPixelTarget(0); //goes to home position
        horizontalElevator.setExtenderLength(0.0);

        //Sets a stopwatch any time there is a pixel in the handoff
        if(scoringMechanism.getPixelColor() != PixelColor.None){
            colorTime = System.currentTimeMillis();
        }else{
            colorTime = 0;
        }

        //Checks if the Horizontal elevator has made it back
        if(horizontalElevator.getExenderLength() > 10) return;

        //Activates the motors to move the pixel
        scoringMechanism.setHandoff(0.5);
        horizontalElevator.setRearIntake(0.5);


    }

    @Override
    public boolean isFinished(){
        return colorTime > 500; //miliseconds
        //TODO: Make sure the pixel has moved far enough up the handoff
    }

    @Override
    public void end(boolean interrupted){
        scoringMechanism.setHandoff(0.0);
        horizontalElevator.setRearIntake(0.0);
    }

}
