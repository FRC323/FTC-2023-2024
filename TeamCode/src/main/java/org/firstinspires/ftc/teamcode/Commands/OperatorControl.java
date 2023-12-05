package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;

public class OperatorControl extends CommandBase {

    ScoringMechanism scoringMechanism;

    private GamepadEx gamepad;

    private final GamepadButton ButtonA =  new GamepadButton(this.gamepad, GamepadKeys.Button.A);
    private final GamepadButton ButtonB =  new GamepadButton(this.gamepad, GamepadKeys.Button.B);
    private final GamepadButton ButtonX =  new GamepadButton(this.gamepad, GamepadKeys.Button.X);
    private final GamepadButton ButtonY =  new GamepadButton(this.gamepad, GamepadKeys.Button.Y);



    public OperatorControl(ScoringMechanism scoringMechanism, Gamepad gamepad){
        this.scoringMechanism = scoringMechanism;
        this.gamepad = new GamepadEx(gamepad);
    }


    @Override
    public void execute(){
        double armAngle = Math.PI;
        double elevatorLength = 0;

        if(ButtonA.get()) armAngle = Math.PI/2;
        if(ButtonB.get()) armAngle = 3 * (Math.PI/2);

        if(ButtonX.get()) elevatorLength = 5;
        if(ButtonY.get()) elevatorLength = 1;

        if(gamepad.getLeftY() > 0.1){
            elevatorLength += gamepad.getLeftY();
        }
        if(gamepad.getRightY() > 0.1){
            elevatorLength += gamepad.getRightY();
        }

        scoringMechanism.setTargetState(elevatorLength,armAngle);


    }



}
