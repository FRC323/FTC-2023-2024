package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalElevator;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    HorizontalElevator m_horizontalElevator;
    GamepadEx gamepad;

    GamepadButton rightBumper;
    GamepadButton leftBumper;

    public DriverController(DriveBase m_driveBase, HorizontalElevator horizontalElevator, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        this.m_horizontalElevator = horizontalElevator;

        this.gamepad = new GamepadEx(gamepad);
        rightBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.LEFT_BUMPER);


        addRequirements(m_horizontalElevator);
        addRequirements(m_driveBase);
    }

    @Override
    public void execute(){

        if(rightBumper.get()) m_driveBase.resetGyro();

        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
            m_horizontalElevator.setIntake(DcMotorSimple.Direction.REVERSE,0.75);
        }else if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
            m_horizontalElevator.setIntake(DcMotorSimple.Direction.FORWARD,0.75);
        }else {
            m_horizontalElevator.setIntake(DcMotorSimple.Direction.REVERSE,0.0);
        }

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        gamepad.getRightY(),
                        gamepad.getRightX(),
                        gamepad.getLeftX()
                ),
                !leftBumper.get()
        );


    }

}
