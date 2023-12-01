package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    GamepadEx driverGamepad;

    GamepadButton rightBumper;
    GamepadButton leftBumper;

    public DriverController(DriveBase m_driveBase, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        driverGamepad = new GamepadEx(gamepad);
        rightBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);

        addRequirements(m_driveBase);
    }

    @Override
    public void execute(){

        if(rightBumper.get()) m_driveBase.resetGyro();

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        driverGamepad.getRightY(),
                        -driverGamepad.getRightX(),
                        driverGamepad.getLeftX()
                ),
                !leftBumper.get()
        );


    }

}
