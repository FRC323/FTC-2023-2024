package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    GamepadEx gamepad;

    GamepadButton rightBumper;
    GamepadButton leftBumper;

    public DriverController(DriveBase m_driveBase, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        this.gamepad = new GamepadEx(gamepad);
        rightBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.LEFT_BUMPER);

        addRequirements(m_driveBase);
    }

    @Override
    public void execute(){

        if(rightBumper.get()) m_driveBase.resetGyro();

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        gamepad.getRightY(),
                        -gamepad.getRightX(),
                        gamepad.getLeftX()
                ),
                !leftBumper.get()
        );


    }

}
