package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ScoringMechanism extends SubsystemBase {
    //Outputs
    private Motor elevatorMotor;
    private Servo rightServo;
    private Servo leftServo;
    private Motor handoffMotor;

    //Inputs
//    private  armEncoder; //TODO: Find out how to read from external encoders
    private DigitalChannel magSwitch;


    //CONSTANTS
    final double ELEVATOR_MAX_DISTANCE = 850; //In encoder ticks
    final double ARM_LENGTH = 12.0; //inches
    final double ELEVATOR_ANGLE = 0.45; //radians

    final double ELEVATOR_DISTANCE_PER_PULSE = 1; //TODO
    final double STARTING_HEIGHT = 0.0; //inches


    private double armAngle = 0.0;
    private PIDFController elevatorController;
    private Telemetry telemetry;


    public ScoringMechanism(HardwareMap hardwareMap, Telemetry telemetry){
        elevatorMotor = new Motor(hardwareMap,"Elevator Motor");
        rightServo = hardwareMap.get(Servo.class,"Right Arm Servo");
        leftServo = hardwareMap.get(Servo.class,"Left Arm Servo");
//        handoffMotor = hardwareMap.get(DcMotor.class,"scoring.motor.handoff");


        magSwitch = hardwareMap.get(DigitalChannel.class,"Mag Switch");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        this.telemetry = telemetry;

        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);


        elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevatorController = new PIDFController(0.005,0.0,0.00,0.0);

    }

    @Override
    public void periodic(){

        //Elevator Movement
        double elevatorPower = elevatorController.calculate(elevatorMotor.getCurrentPosition());

        //Checks if reaching lower bound of elevator
        if(!magSwitch.getState() && elevatorPower < 0.0){
            elevatorPower = -0.15;
        }

        //Checks if reaching upper bound of elevator
        if(elevatorMotor.getCurrentPosition() > ELEVATOR_MAX_DISTANCE && elevatorPower > 0.0) {
            elevatorPower = 0.0;
        }

        elevatorMotor.set(elevatorPower);


        //Arm Movement
        double position = armAngle / (2 * Math.PI);

        leftServo.setPosition(position);
        rightServo.setPosition(position);


        telemetry.addLine(String.format("Pos:%d",elevatorMotor.getCurrentPosition()));
        telemetry.addLine(String.format("Mag: %b", !magSwitch.getState()));
        telemetry.update();

    }

    public void setTargetState(double elevatorLength, double armAngle){
        this.armAngle = armAngle;
        elevatorController.setSetPoint(elevatorLength);
    }



    public void setTargetEndPoint(double distance, double height){
        double[][] stateOptons =  getStateOptionsFromTargetPoint(distance,height);
    }

    public void resetElevatorEncoder(){
        elevatorMotor.resetEncoder();
    }

    public double getElevatorLength(){
        return elevatorMotor.getCurrentPosition() * ELEVATOR_DISTANCE_PER_PULSE;
    }
    public  double getArmAngle(){
        //TODO:Return actual arm angle
        return 0.0;
    }

    public double getCurrentHeight(){
        double armHeight = sin(getArmAngle()) * ARM_LENGTH;
        double elevatorHeight = sin(ELEVATOR_ANGLE) * getElevatorLength();
        return elevatorHeight + armHeight + STARTING_HEIGHT;
    }

    public double getCurrentDistance(){
        double armDistance = cos(getArmAngle()) * ARM_LENGTH;
        double elevatorDistance = cos(ELEVATOR_ANGLE) * getElevatorLength();
        return elevatorDistance + armDistance;
    }



    @Nullable
    public double[][] getStateOptionsFromTargetPoint(double distance,double height){
        /*  Potential Points for Joint from end Point are on:
            - (x-d)^2 + (y - h)^2 = ARM_LENGTH^2 (Circle Formula)
            Potential Points on Elevator:
            - m = tan(ARM_ANGLE)
            - y = mx

            Solving the System of Equations:
            - (x-d)^2 + (mx - h)^2 = ARM_LENGTH^2
            - [x^2 -2dx + d^2] + [(mx)^2 -2mhx + h^2] = ARM_LENGTH^2
            - (1 + m^2)(x^2) -2x(d + mh) = ARM_LENGTH^2 - h^2 - d^2
            - Get X Solutions using quadratic formula
            - Get Y Coordinates using y = mx
         */

        double h = height - STARTING_HEIGHT;
        double d = distance;

        final double m = Math.tan(ELEVATOR_ANGLE);
        double a = 1 + Math.pow(m,2);
        double b = -2 * (d + (m*h));
        double c = -(Math.pow(ARM_LENGTH,2) - Math.pow(h,2) - Math.pow(d,2));
        double[] xCoordinates =  solveQuadratic(a,b,c);

        double[][] possibleStates = {
                getStateFromJointCordinate(xCoordinates[0],distance),
                getStateFromJointCordinate(xCoordinates[1],distance)
        };

        if(possibleStates[0] == null || possibleStates[1] == null) return null;
        return possibleStates;


    }

    @Nullable
    private double[] getStateFromJointCordinate(double x, double targetX){
        //Get using pythagorean theorem
        double elevatorDistance = Math.sqrt(Math.pow(x,2) + Math.pow(x * Math.tan(ELEVATOR_ANGLE),2));

        // sin(armAngle) = (y - targetY) / ARM_LENGTH
        double armAngle = Math.acos( (x - targetX) / ARM_LENGTH);

        if(elevatorDistance == Double.NaN || armAngle == Double.NaN) return null;

        return new double[] {
                elevatorDistance,
                armAngle,
        };
    }

    @Nullable
    private  double[] solveQuadratic(double a, double b, double c){
        double[] solutions = {0.0,0.0};
        solutions[0] = (-b + Math.sqrt( Math.pow(b,2) - (3*a*c) )) / (2*a);
        solutions[1] = (-b - Math.sqrt( Math.pow(b,2) - (3*a*c) )) / (2*a);
        if(solutions[0] == Double.NaN || solutions[1] == Double.NaN) return null;
        return solutions;
    }




    //Determining fastest position;
    /*
    knowns: ARM_SPEED, ELEVATOR_SPEED, height, distance

    Time = Max(arm_distance/ARM_SPEED,elevator_distance/ELEVATOR_SPEED);
    //Stategies
    - Maximize the distance moved towards the target at any given time.
    - Minimize the distance of the arm because it will take the longest amount of time.

    armAngle = arcsin( [height - (sin(ELEVATOR_ANGLE) * elevatorLength)] / ARM_LENGTH )
    armAngle = arccos( [distance - (cos(ELEVATOR_ANGLE) * elevatorLength)] / ARM_LENGTH )

    elevatorLen = [height - (cos(armAngle * ARM_LENGTH)] / cos(ELEVATOR_ANGLE)
    elevatorLen = [distance - (sin(armAngle * ARM_LENGTH)] / sin(ELEVATOR_ANGLE)

    distance = [cos(armAngle) * ARM_LENGTH] + [cos(ELEVATOR_ANGLE) * elevatorLength];
    height = [sin(armAngle) * ARM_LENGTH] + [sin(ELEVATOR_ANGLE) * elevatorLength];

    //Solve system of equations for elevatorLen from height and distance
    //This should result in two points that are between 0 and 2pi
    //figure out time that is takes to go to either point
        - Also check if both points are possible to go to.

     */


    /*
        If given a target distance and height for the end point
        - draw a circle of radius = ARM_LENGTH around the end point
        - draw a straight line at the angle of the elevator
        - There will only ever be 2 points at max where the line intersects the circle
        - These intersections are the possible locations of the arm elevator connection
        - This intersection gives you the length of the elevator and the angle of the arm


        The points are the intersections between these two equations
        - (x-distance)^2 + (y-height)^2 = ARM_LENGTH^2 (note: renamed circle equation)
        - tan(ELEVATOR_ANGLE)x + ELEVATOR_START_HEIGHT

     */


}
