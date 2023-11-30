package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    GamepadEx driverGamepad;


    public DriverController(DriveBase m_driveBase, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        driverGamepad = new GamepadEx(gamepad);

        addRequirements(m_driveBase);
    }

    @Override
    public void execute(){

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        driverGamepad.getRightX(),
                        driverGamepad.getRightY(),
                        driverGamepad.getLeftY()
                ),
                true
        );


    }

}
