package org.firstinspires.ftc.teamcode.Commands.Controllers;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PixelClaw;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    Launcher m_launcher;
    PixelClaw m_pixelClaw;

    GamepadEx gamepad;

    GamepadButton rightBumper;
    GamepadButton leftBumper;


    public DriverController(DriveBase m_driveBase,Launcher launcher,PixelClaw pixelClaw, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        this.m_launcher = launcher;
        this.m_pixelClaw = pixelClaw;

        this.gamepad = new GamepadEx(gamepad);
        rightBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.LEFT_BUMPER);


        addRequirements(m_launcher);
        addRequirements(m_driveBase);
    }

    @Override
    public void execute() {

        if (rightBumper.get()) m_driveBase.resetGyro();

        if (gamepad.getButton(GamepadKeys.Button.A)) m_driveBase.resetOdometry();

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        gamepad.getRightX(),
                        gamepad.getRightY(),
                        gamepad.getLeftX()
                ),
                !leftBumper.get()
        );

//        if(gamepad.getButton(GamepadKeys.Button.X)){
//            m_pixelClaw.setLeftClosed();
//        }
//
//        if(gamepad.getButton(GamepadKeys.Button.Y)){
//            m_pixelClaw.setLeftOpen();
//            //for Redundancy
//        }
//
//        if(gamepad.getButton(GamepadKeys.Button.A)){
//            m_pixelClaw.setRightClosed();
//        }
//
//        if(gamepad.getButton(GamepadKeys.Button.B)){
//            m_pixelClaw.setRightOpen();
//        }
//
//        if(gamepad.getButton(GamepadKeys.Button.START)){
//            m_launcher.setLaunched();
//        }
//
//        if(gamepad.getButton(GamepadKeys.Button.BACK)){
//            m_launcher.setBack();
//        }

    }

}
