package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriverController;
import org.firstinspires.ftc.teamcode.Subsystems.DriveBase;



@TeleOp(name = "Drive Base Only", group = "tests")
public class DriveBaseOnly extends CommandOpMode {

    @Override
    public void initialize() {
        DriveBase m_driveBase = new DriveBase(hardwareMap);

        DriverController m_driverCommand = new DriverController(m_driveBase,gamepad1);

        schedule(m_driverCommand);
    }

}
