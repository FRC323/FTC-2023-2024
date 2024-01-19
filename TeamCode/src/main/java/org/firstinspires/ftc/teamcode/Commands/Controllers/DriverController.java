package org.firstinspires.ftc.teamcode.Commands.Controllers;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalElevator;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    HorizontalElevator m_horizontalElevator;
    Launcher m_launcher;

    GamepadEx gamepad;

    GamepadButton rightBumper;
    GamepadButton leftBumper;


    public DriverController(DriveBase m_driveBase, HorizontalElevator horizontalElevator,Launcher launcher, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        this.m_horizontalElevator = horizontalElevator;
        this.m_launcher = launcher;

        this.gamepad = new GamepadEx(gamepad);
        rightBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.LEFT_BUMPER);


        addRequirements(m_horizontalElevator);
        addRequirements(m_launcher);
        addRequirements(m_driveBase);
    }

    @Override
    public void execute() {

        if (rightBumper.get()) m_driveBase.resetGyro();

        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            m_horizontalElevator.setFrontIntake(1.0);
            m_horizontalElevator.setRearIntake(1.0);
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            m_horizontalElevator.setFrontIntake(-1.0);
        } else {
            m_horizontalElevator.setRearIntake(0.0);
            m_horizontalElevator.setFrontIntake(0.0);
        }

        if (gamepad.getButton(GamepadKeys.Button.A)) m_driveBase.resetOdometry();

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        gamepad.getRightX(),
                        gamepad.getRightY(),
                        gamepad.getLeftX()
                ),
                !leftBumper.get()
        );


        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            m_horizontalElevator.setIntakePosition(HorizontalElevator.IntakePose.GROUND);
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            m_horizontalElevator.setIntakePosition(HorizontalElevator.IntakePose.VERTICAL);
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)){
            m_horizontalElevator.setIntakePosition(HorizontalElevator.IntakePose.STARTING);
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            m_horizontalElevator.setIntakePosition(HorizontalElevator.IntakePose.HORIZONTAL);
        }

        m_horizontalElevator.setExtenderPower(gamepad.getLeftY());

        if(gamepad.getButton(GamepadKeys.Button.START)){
            m_launcher.setLaunched();
        }

        if(gamepad.getButton(GamepadKeys.Button.BACK)){
            m_launcher.setBack();
        }

    }

}
