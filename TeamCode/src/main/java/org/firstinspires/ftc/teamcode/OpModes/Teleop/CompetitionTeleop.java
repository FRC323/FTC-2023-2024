package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.DriverController;
import org.firstinspires.ftc.teamcode.Commands.OperatorController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalElevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;

@TeleOp(name = "Main", group = "competition")
public class CompetitionTeleop extends CommandOpMode {
    //Subsystems
    private DriveBase m_driveBase;
    private HorizontalElevator m_horizontalElevator;
    private ScoringMechanism m_scoringMechanism;

    //Commands
    private DriverController driverController;
    private OperatorController operatorController;

    @Override
    public void initialize() {
        m_driveBase = new DriveBase(hardwareMap,telemetry);
        m_horizontalElevator = new HorizontalElevator(hardwareMap,telemetry);
        m_scoringMechanism = new ScoringMechanism(hardwareMap,telemetry);

        driverController = new DriverController(m_driveBase,m_horizontalElevator,gamepad1);
        operatorController = new OperatorController(m_scoringMechanism,gamepad2);


        schedule(driverController);
        schedule(operatorController);
    }
}
