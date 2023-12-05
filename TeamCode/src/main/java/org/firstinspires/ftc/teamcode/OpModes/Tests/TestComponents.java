package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriverController;
import org.firstinspires.ftc.teamcode.Commands.OperatorControl;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;


@TeleOp(name = "Testing Components", group = "tests")
public class TestComponents extends CommandOpMode {

    @Override
    public void initialize() {
        DriveBase m_driveBase = new DriveBase(hardwareMap, telemetry);
        ScoringMechanism m_scoringMechanism = new ScoringMechanism(hardwareMap);

        DriverController m_driverCommand = new DriverController(m_driveBase,gamepad1);
        OperatorControl m_operatorCommand = new OperatorControl(m_scoringMechanism,gamepad2);

        schedule(m_driverCommand);
        schedule(m_operatorCommand);
    }



}
