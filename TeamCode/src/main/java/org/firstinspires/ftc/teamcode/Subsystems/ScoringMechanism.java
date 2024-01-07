package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Optional;
import java.util.OptionalDouble;


public class ScoringMechanism extends SubsystemBase {
    //Outputs
    private final Motor elevatorMotor;
    private final Servo rightServo;
    private final Servo leftServo;
    private final CRServo handoffMotor;

    //Inputs
//    private  armEncoder; //TODO: Find out how to read from external encoders
    private final DigitalChannel magSwitch;
    private final Motor.Encoder armEncoder;

    //CONSTANTS
    private final double ARM_LENGTH = 13.75; //inches
    private final double ARM_MIN_ANGLE = 0.0; //Radians
    private final double ARM_MAX_ANGLE = PI; //Radians TODO
    private final double ARM_ANGLE_TO_POSE = 1.0/ARM_MAX_ANGLE;//TODO

    //Elevator Constants
    final double ELEVATOR_MAX_DISTANCE = 13.5; //Inches
    final double ELEVATOR_SLOPE = Math.sqrt(3); //30 - 60 -90 triangle;
    final double ELEVATOR_DISTANCE_PER_PULSE = 5.5/325.0;

    //Variables
    private double armAngle = 0.0;
    private final PIDFController elevatorController;
    private double handoffPower = 0.0;
    private DcMotorSimple.Direction handoffDirection = DcMotorSimple.Direction.REVERSE;


    private final Telemetry telemetry;


    public ScoringMechanism(HardwareMap hardwareMap, Telemetry telemetry){
        elevatorMotor = new Motor(hardwareMap,"Elevator");
        rightServo = hardwareMap.get(Servo.class,"Arm 1");
        leftServo = hardwareMap.get(Servo.class,"Arm 2");
        handoffMotor = hardwareMap.get(CRServo.class,"Handoff");

        magSwitch = hardwareMap.get(DigitalChannel.class,"Elevator Zero");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        armEncoder = new Motor(hardwareMap,"Back Right/Arm Encoder").encoder;

        this.telemetry = telemetry;

        //Arm Config
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);


        //Elevator Config
        elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        elevatorMotor.setInverted(true);

        elevatorController = new PIDFController(0.02,0.0,0.00,0.0);
        elevatorController.setTolerance(15);


    }

    @Override
    public void periodic(){

        //Elevator Movement
        double elevatorPower = elevatorController.calculate(elevatorMotor.getCurrentPosition());

        //Checks if reaching lower bound of elevator
        if(!magSwitch.getState() && elevatorPower < 0.0){
            elevatorPower = -0.2;
        }

        //Checks if reaching upper bound of elevator
        if((elevatorMotor.getCurrentPosition() * ELEVATOR_DISTANCE_PER_PULSE)  > ELEVATOR_MAX_DISTANCE && elevatorPower > 0.0) {
            elevatorPower = 0.0;
        }

        if(elevatorController.atSetPoint()) elevatorPower = 0.0;

        elevatorMotor.set(elevatorPower);


        //Arm Movement
        OptionalDouble position = getServoPoseFromAngle(armAngle);

        if(position.isPresent()) {
            leftServo.setPosition(position.getAsDouble());
            rightServo.setPosition(position.getAsDouble());
        }

        //Handoff
        handoffMotor.setPower(handoffPower);
        handoffMotor.setDirection(handoffDirection);

        telemetry.addLine(String.format("Ticks:%d",elevatorMotor.encoder.getPosition()));

    }

    //Setters
    public void setTargetState(double elevatorLength, double armAngle){
        this.armAngle = armAngle;
        elevatorController.setSetPoint(elevatorLength);
    }
    public void setHandoffState(double power, DcMotorSimple.Direction direction){
        handoffPower = power;
        handoffDirection = direction;
    }
    public boolean setTargetEndPoint(double distance, double height){
        Optional<double[]> stateOptional =  getStateFromTargetPoint(distance,height);


        if(!stateOptional.isPresent()) return false;

        double[] state = stateOptional.get();

        elevatorController.setSetPoint(state[0]);
        armAngle = state[1];

        return true;
    }
    public void resetElevatorEncoder(){
        elevatorMotor.resetEncoder();
    }


    //Getters
    public double getElevatorLength(){
        return elevatorMotor.getCurrentPosition() * ELEVATOR_DISTANCE_PER_PULSE;
    }
    public  double getArmAngle(){
        //TODO:Return actual arm angle
        return 0.0;
    }

//    public double getCurrentHeight(){
//        double armHeight = sin(getArmAngle()) * ARM_LENGTH;
//        double elevatorHeight = sin(ELEVATOR_ANGLE) * getElevatorLength();
//        return elevatorHeight + armHeight + STARTING_HEIGHT;
//    }
//
//    public double getCurrentDistance(){
//        double armDistance = cos(getArmAngle()) * ARM_LENGTH;
//        double elevatorDistance = cos(ELEVATOR_ANGLE) * getElevatorLength();
//        return elevatorDistance + armDistance;
//    }

    private OptionalDouble getServoPoseFromAngle(double angle){
        if(angle > ARM_MAX_ANGLE  || angle < ARM_MIN_ANGLE){
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(
                angle * ARM_ANGLE_TO_POSE
        );
    }


    //Arm State from target height and distance of end-effector

    private Optional<double[]> getStateFromTargetPoint(double distance, double height){
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

        double a = 1 + Math.pow(ELEVATOR_SLOPE,2);
        double b = -2 * (distance + (ELEVATOR_SLOPE*height));
        double c = -(Math.pow(ARM_LENGTH,2) - Math.pow(height,2) - Math.pow(distance,2));
        double[] xCoordinates =  solveQuadratic(a,b,c);


        ArrayList<double[]> possibleStates = new ArrayList<>();

        //Adds
        getStateFromJointCordinate(xCoordinates[0],distance).ifPresent(possibleStates::add);
        getStateFromJointCordinate(xCoordinates[1],distance).ifPresent(possibleStates::add);



        return getBestStatePossibility(possibleStates);
    }
    private Optional<double[]> getStateFromJointCordinate(double x, double distance){
        //Get using pythagorean theorem
        double elevatorDistance = Math.sqrt(Math.pow(x,2) + Math.pow(x * ELEVATOR_SLOPE,2));

        // cos(armAngle) = (x - targetX) / ARM_LENGTH
        double armAngle = Math.acos( (x - distance) / ARM_LENGTH); //TODO: Understand this better

        if(Double.isNaN(elevatorDistance) || Double.isNaN(armAngle)) return Optional.empty();

        return Optional.of(new double[] {
                elevatorDistance,
                armAngle,
        });
    }


    private  double[] solveQuadratic(double a, double b, double c){
        double[] solutions = {0.0,0.0};
        solutions[0] = (-b + Math.sqrt( Math.pow(b,2) - (3*a*c) )) / (2*a);
        solutions[1] = (-b - Math.sqrt( Math.pow(b,2) - (3*a*c) )) / (2*a);

        return solutions;
    }

    private Optional<double[]> getBestStatePossibility(ArrayList<double[]> states){
        ArrayList<double[]> possibleStates = new ArrayList<>();

        for(double[] state : states){
            //Checks if state is null
            if(Double.isNaN(state[0]) || Double.isNaN(state[1])) continue;

            //Checks that Elevator State is within min/max distance
            if( state[0] < 0 || state[0] > ELEVATOR_MAX_DISTANCE) continue;

            if(state[1] < ARM_MIN_ANGLE || state[1] > ARM_MAX_ANGLE) continue;

            //TODO: Guarantee that there is only ever one or less states

            possibleStates.add(state);
        }

        if(possibleStates.size() == 1) return Optional.of(possibleStates.get(0));
        else return Optional.empty();
    }

    /* Elevator->Arm joint position solution
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


    //Determining fastest position brainstorming;
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




}
