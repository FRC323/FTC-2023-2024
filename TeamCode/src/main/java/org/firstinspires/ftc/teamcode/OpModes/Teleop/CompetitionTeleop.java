package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Controllers.DriverController;
import org.firstinspires.ftc.teamcode.Commands.InformationProcessing;
import org.firstinspires.ftc.teamcode.Commands.Controllers.OperatorController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalElevator;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

@TeleOp(name = "Main", group = "competition")
public class CompetitionTeleop extends CommandOpMode {
    //Subsystems
    private DriveBase m_driveBase;
    private HorizontalElevator m_horizontalElevator;
    private ScoringMechanism m_scoringMechanism;
    private VisionSubsystem m_visiionSubstem;

    //Commands
    private DriverController driverController;
    private OperatorController operatorController;
    private InformationProcessing informationProcessing;

    @Override
    public void initialize() {
        m_driveBase = new DriveBase(hardwareMap,telemetry);
        m_horizontalElevator = new HorizontalElevator(hardwareMap,telemetry);
        m_scoringMechanism = new ScoringMechanism(hardwareMap,telemetry);
        m_visiionSubstem = new VisionSubsystem(hardwareMap,telemetry);

        driverController = new DriverController(m_driveBase,m_horizontalElevator,gamepad1);
        operatorController = new OperatorController(m_scoringMechanism,gamepad2);
        informationProcessing = new InformationProcessing(m_visiionSubstem,telemetry);

        schedule(driverController);
        schedule(operatorController);
        schedule(informationProcessing);
    }
}
