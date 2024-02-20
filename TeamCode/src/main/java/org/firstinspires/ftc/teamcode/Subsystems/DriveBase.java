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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveBase extends SubsystemBase{

    //Motors
    private final Motor frontRightMotor;
    private final Motor frontLeftMotor;
    private final Motor backRightMotor;
    private final Motor backLeftMotor;

    private final Motor.Encoder leftEncoder;
    private final Motor.Encoder centerEncoder;
    private final Motor.Encoder rightEncoder;


    // The lateral distance between the left and right odometers
    // is called the track width. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 15.533;//in //14.7;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 8.412;//in //-2.1;

    private final double ODOMETRY_DISTANCE_PER_TICK = 23.625/28167; //1 turn = 8180

    private final IMU imu;

    //Drive, kinematics, and Odometry
    private final MecanumDrive m_drive;
    private final HolonomicOdometry m_odometry;
    private Rotation2d gyroOdometryResetAngle = new Rotation2d(0.0 );
    private boolean fieldCentric;

    private ChassisSpeeds speeds = new ChassisSpeeds(0.0,0.0,0.0);

    private final Telemetry telemetry;

    public DriveBase(HardwareMap hardware_map, Telemetry telemetry){

        //Motors
        frontLeftMotor = new Motor(hardware_map,"Front Left");
        frontRightMotor = new Motor(hardware_map,"Front Right");
        backLeftMotor = new Motor(hardware_map,"Back Left");
        backRightMotor = new Motor(hardware_map,"Back Right/Right Odom");

        this.telemetry = telemetry;

        //Encoders
        leftEncoder = new Motor(hardware_map, "Left Odom").encoder;
        rightEncoder = backRightMotor.encoder;
        centerEncoder = new Motor(hardware_map, "Center Odom").encoder;

        leftEncoder.setDistancePerPulse(ODOMETRY_DISTANCE_PER_TICK);
        leftEncoder.setDirection(Motor.Direction.REVERSE);
        rightEncoder.setDistancePerPulse(ODOMETRY_DISTANCE_PER_TICK);
        rightEncoder.setDirection(Motor.Direction.FORWARD);
        centerEncoder.setDistancePerPulse(ODOMETRY_DISTANCE_PER_TICK);
        centerEncoder.setDirection(Motor.Direction.REVERSE);

        //Gyro
        imu = hardware_map.get(IMU.class,"imu");

        //Kinematics
        m_drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);
        m_drive.setRightSideInverted(false);

        //Odometry
        m_odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                centerEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

    }

    @Override
    public void periodic(){

        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(false);
        backLeftMotor.setInverted(false);
        backRightMotor.setInverted(true);


        if(fieldCentric){
            m_drive.driveFieldCentric(
                    speeds.vxMetersPerSecond,
                    -speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    getRotation().getDegrees()
            );
        }else{

            m_drive.driveRobotCentric(
                    speeds.vxMetersPerSecond,
                    -speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond
            );


        }


        m_odometry.updatePose();
//        telemetry.addLine(String.format("X: %4.2f, Y:%4.2f",m_odometry.getPose().getX(),m_odometry.getPose().getY()));
//        telemetry.addLine(String.format("L:%4.2f,R:%4.2f,C%4.2f",leftEncoder.getDistance(),rightEncoder.getDistance(),centerEncoder.getDistance()));
    }

    public void setDriveSpeeds(ChassisSpeeds speeds, boolean fieldCentric){
        this.speeds = speeds;
        this.fieldCentric = fieldCentric;
    }
    public Pose2d getRobotPose2d(){
        return new Pose2d(
                m_odometry.getPose().getX(),
                m_odometry.getPose().getY(),
                getRotation().minus(gyroOdometryResetAngle));
    }
    public Rotation2d getRotation(){
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
    public void setRobotPose(Pose2d robotPose){
        m_odometry.updatePose(robotPose);
    }
    public void resetGyro(){
        this.imu.resetYaw();
    }
    public void resetOdometry(){
        gyroOdometryResetAngle = getRotation();
        m_odometry.updatePose(new Pose2d(0.0,0.0,new Rotation2d(0.0)));
        leftEncoder.reset();
        rightEncoder.reset();
        centerEncoder.reset();
    }
}
