package org.firstinspires.ftc.teamcode.Commands.Controllers;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PixelClaw;

public class DriverController extends CommandBase {

    DriveBase m_driveBase;
    Launcher m_launcher;
    PixelClaw m_pixelClaw;
    Arm m_arm;

    GamepadEx gamepad;

    GamepadButton rightBumper;
    GamepadButton leftBumper;


    public DriverController(DriveBase m_driveBase, Launcher launcher, PixelClaw pixelClaw, Arm arm, Gamepad gamepad){
        this.m_driveBase = m_driveBase;
        this.m_launcher = launcher;
        this.m_pixelClaw = pixelClaw;
        this.m_arm = arm;

        this.gamepad = new GamepadEx(gamepad);
        rightBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(this.gamepad, GamepadKeys.Button.LEFT_BUMPER);


        addRequirements(m_launcher);
        addRequirements(m_driveBase);
        addRequirements(m_pixelClaw);
        addRequirements(m_arm);
    }

    @Override
    public void execute() {

        if (rightBumper.get()) m_driveBase.resetGyro();

//        if (gamepad.getButton(GamepadKeys.Button.A)) m_driveBase.resetOdometry();

        m_driveBase.setDriveSpeeds(
                new ChassisSpeeds(
                        gamepad.getRightX(),
                        gamepad.getRightY(),
                        gamepad.getLeftX()
                ),
                false
        );

        if(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            m_arm.setArmMotorPower(gamepad.getLeftY());
            m_arm.setTargetRads(m_arm.getAngleRads());
        }

        if(gamepad.getButton(GamepadKeys.Button.Y)) m_arm.setTargetRads(m_arm.HIGH_SCORE);

        if(gamepad.getButton(GamepadKeys.Button.X)) m_arm.setTargetRads(m_arm.INTAKE_DOWN);

        if(gamepad.getButton(GamepadKeys.Button.A)) m_arm.setTargetRads(m_arm.CLIMB_UP);

        if(gamepad.getButton(GamepadKeys.Button.B)) m_arm.setTargetRads(m_arm.LOW_SCORE);

        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
            m_pixelClaw.setLeftClosed();
        }

        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            m_pixelClaw.setRightClosed();
        }

        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            m_pixelClaw.setRightOpen();
            m_pixelClaw.setLeftOpen();
        }

        if(gamepad.getButton(GamepadKeys.Button.START)){
            m_launcher.setLaunched();
        }

        if(gamepad.getButton(GamepadKeys.Button.BACK)){
            m_launcher.setBack();
        }

    }

}
