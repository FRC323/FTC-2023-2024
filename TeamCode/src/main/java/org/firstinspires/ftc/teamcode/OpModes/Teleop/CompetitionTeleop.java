package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Controllers.DriverController;
import org.firstinspires.ftc.teamcode.Commands.Controllers.OperatorController;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PixelClaw;

@TeleOp(name = "Main", group = "competition")
public class CompetitionTeleop extends CommandOpMode {
    //Subsystems
    private DriveBase m_driveBase;
    private Launcher m_launcher;
    private PixelClaw m_pixelClaw;
    private Arm m_arm;

    //Commands
    private DriverController driverController;
//    private OperatorController operatorController;

    @Override
    public void initialize() {
        m_driveBase = new DriveBase(hardwareMap,telemetry);
        m_launcher = new Launcher(hardwareMap);
        m_pixelClaw = new PixelClaw(hardwareMap);
        m_arm = new Arm(hardwareMap,telemetry);

        driverController = new DriverController(m_driveBase,m_launcher,m_pixelClaw,m_arm, gamepad1);
//        operatorController = new OperatorController(gamepad2);

        schedule(driverController);
//        schedule(operatorController);
    }
}
