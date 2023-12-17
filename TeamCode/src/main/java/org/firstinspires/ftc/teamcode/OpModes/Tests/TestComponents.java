package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Controllers.OperatorController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringMechanism;


@TeleOp(name = "Testing Components", group = "tests")
@Disabled
public class TestComponents extends CommandOpMode {

    @Override
    public void initialize() {
        DriveBase m_driveBase = new DriveBase(hardwareMap, telemetry);
        ScoringMechanism m_scoringMechanism = new ScoringMechanism(hardwareMap, telemetry);

//        DriverController m_driverCommand = new DriverController(m_driveBase,gamepad1);
        OperatorController m_operatorCommand = new OperatorController(m_scoringMechanism,gamepad2);

//        schedule(m_driverCommand);
        schedule(m_operatorCommand);
    }



}
