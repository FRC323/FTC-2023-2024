package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveBase extends SubsystemBase{

    //Motors
    private final Motor frontRightMotor;
    private final Motor frontLeftMotor;
    private final Motor backRightMotor;
    private final Motor backLeftMotor;

//    private final Motor.Encoder LeftEncoder;
//    private final Motor.Encoder CenterEncoder;
//    private final Motor.Encoder RightEncoder;

    //TODO: Get actual track width and wheel offset

    // The lateral distance between the left and right odometers
    // is called the track width. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 14.7;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -2.1;


//    private final IMU imu;

    //Drive, kinematics, and Odometry
    private final MecanumDrive m_drive;
//    private final HolonomicOdometry m_odometry;
    private boolean fieldCentric;

    private ChassisSpeeds speeds = new ChassisSpeeds(0.0,0.0,0.0);



    public DriveBase(HardwareMap hardware_map){

        frontLeftMotor = new Motor(hardware_map,"Front Left");
        frontRightMotor = new Motor(hardware_map,"Front Right");
        backLeftMotor = new Motor(hardware_map,"Back Left");
        backRightMotor = new Motor(hardware_map,"Back Right");



//        LeftEncoder = hardware_map.get(Motor.Encoder.class, "LeftEncoder").setDistancePerPulse(0);
//        RightEncoder = hardware_map.get(Motor.Encoder.class, "RightEncoder").setDistancePerPulse(0);
//        CenterEncoder = hardware_map.get(Motor.Encoder.class, "CenterEncoder").setDistancePerPulse(0);

//        imu = hardware_map.get(IMU.class,"imu");



        m_drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);

//        m_odometry = new HolonomicOdometry(
//                LeftEncoder::getDistance,
//                RightEncoder::getDistance,
//                CenterEncoder::getDistance,
//                TRACKWIDTH,
//                CENTER_WHEEL_OFFSET
//        );

    }

    @Override
    public void periodic(){

//        Rotation2d gyroAngle = Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

//        if(fieldCentric){
//            m_drive.driveFieldCentric(
//                    speeds.vyMetersPerSecond,
//                    speeds.vxMetersPerSecond,
//                    speeds.omegaRadiansPerSecond,
//                    gyroAngle.getDegrees()
//            );
//        }else{

            m_drive.driveRobotCentric(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond
            );
//        }


//        m_odometry.updatePose();
    }

    public void setDriveSpeeds(ChassisSpeeds speeds, boolean fieldCentric){
        this.speeds = speeds;
        this.fieldCentric = fieldCentric;
    }
//    public Pose2d getRobotPose2d(){
//        return m_odometry.getPose();
//    }
//    public void setRobotPose(Pose2d robotPose){
//        m_odometry.updatePose(robotPose);
//    }

}
