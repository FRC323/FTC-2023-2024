package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;

public class OperatorController extends CommandBase {

    ScoringMechanism scoringMechanism;

    private GamepadEx gamepad;


    private double elevatorPosition = 0.0;
    private double armAngle = 0.0;


    public OperatorController(ScoringMechanism scoringMechanism, Gamepad gamepad){
        this.gamepad = new GamepadEx(gamepad);
        this.scoringMechanism = scoringMechanism;
    }


    @Override
    public void execute(){

        if(gamepad.getButton(GamepadKeys.Button.A)) armAngle = Math.PI/2;
        if(gamepad.getButton(GamepadKeys.Button.B)) armAngle = 3 * (Math.PI/2);

        if(gamepad.getButton(GamepadKeys.Button.X)) elevatorPosition = 800;
        if(gamepad.getButton(GamepadKeys.Button.Y)) elevatorPosition = 400;

        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            elevatorPosition = 0;
            armAngle = 0.0;
        }



        if(gamepad.getRightY() > 0.05 || gamepad.getRightY() < -0.05){
            elevatorPosition += -10*((gamepad.getRightY()*2)-1);
        }

        if(gamepad.getLeftY() > 0.05 || gamepad.getLeftY() < -0.05){
            armAngle = 3*Math.PI*gamepad.getLeftY();
        }

        if(gamepad.getButton(GamepadKeys.Button.START)){
            elevatorPosition = 0.0;
            scoringMechanism.resetElevatorEncoder();
        }

        scoringMechanism.setTargetState(elevatorPosition,armAngle);


    }



}
