package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ScoringMechanism extends SubsystemBase {

    private Motor elevatorMotor;
    private Servo rightServo;
    private Servo leftServo;
    private Motor handoffMotor;

    //CONSTANTS
    final double ARM_LENGTH = 12.0; //inches
    final double ELEVATOR_ANGLE = 0.45; //radians

    final double ELEVATOR_DISTANCE_PER_PULSE = 1; //TODO
    final double STARTING_HEIGHT = 0.0; //inches

    private double armAngle = 0.0;

    public ScoringMechanism(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(Motor.class,"scoring.motor.elevator");
        rightServo = hardwareMap.get(Servo.class,"scoring.servo.right");
        leftServo = hardwareMap.get(Servo.class,"scoring.servo.right");
        handoffMotor = hardwareMap.get(Motor.class,"scoring.motor.handoff");

        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);

        elevatorMotor.setRunMode(Motor.RunMode.PositionControl);
        elevatorMotor.setPositionTolerance(13.6);
    }

    @Override
    public void periodic(){

        double position = armAngle / (2 * Math.PI);

        rightServo.setPosition(position);
        leftServo.setPosition(position);

        if(!elevatorMotor.atTargetPosition()) elevatorMotor.set(0.5);
        else elevatorMotor.stopMotor();


    }

    public void setTargetState(double elevatorLength, double armAngle){
        this.armAngle = armAngle;

        elevatorMotor.setTargetPosition((int)(elevatorLength / ELEVATOR_DISTANCE_PER_PULSE));
    }






    public void setTargetEndPoint(double distance, double height){
        //TODO: Figure out math for converting desired height and distance to arm pose and elevator pose
        double[][] stateOptons =  getStateOptionsFromTargetPoint(distance,height);



    }

    public double getElevatorLength(){
        //TODO:Return actual elevator length
        return 0.0;
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

        return new double[][] {
                getStateFromJointCordinate(xCoordinates[0],distance),
                getStateFromJointCordinate(xCoordinates[1],distance)
        };


    }

    private double[] solveQuadratic(double a, double b, double c){
        double[] solutions = {0.0,0.0};
        solutions[0] = (-b + Math.sqrt( Math.pow(b,2) - (3*a*c) )) / (2*a);
        solutions[1] = (-b - Math.sqrt( Math.pow(b,2) - (3*a*c) )) / (2*a);
        return solutions;
    }

    private double[] getStateFromJointCordinate(double x, double targetX){
        //Get using pythagorean theorem
        double elevatorDistance = Math.sqrt(Math.pow(x,2) + Math.pow(x * Math.tan(ELEVATOR_ANGLE),2));

        // sin(armAngle) = (y - targetY) / ARM_LENGTH
        double armAngle = Math.acos( (x - targetX) / ARM_LENGTH);

        return new double[] {
            elevatorDistance,
            armAngle,
        };
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
