package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Controllers.DriverController;
import org.firstinspires.ftc.teamcode.Commands.Controllers.OperatorController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;

@TeleOp(name = "Main", group = "competition")
public class CompetitionTeleop extends CommandOpMode {
    //Subsystems
    private DriveBase m_driveBase;
    private Launcher m_launcher;

    //Commands
    private DriverController driverController;
    private OperatorController operatorController;

    @Override
    public void initialize() {
        m_driveBase = new DriveBase(hardwareMap,telemetry);
        m_launcher = new Launcher(hardwareMap);

        driverController = new DriverController(m_driveBase,m_launcher,gamepad1);
        operatorController = new OperatorController(gamepad2);

        schedule(driverController);
        schedule(operatorController);
    }
}
