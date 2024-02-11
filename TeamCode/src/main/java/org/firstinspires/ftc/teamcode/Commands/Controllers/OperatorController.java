package org.firstinspires.ftc.teamcode.Commands.Controllers;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OperatorController extends CommandBase {

    private final GamepadEx gamepad;

    public OperatorController(Gamepad gamepad){
        this.gamepad = new GamepadEx(gamepad);
    }


    @Override
    public void execute(){

    }



}
