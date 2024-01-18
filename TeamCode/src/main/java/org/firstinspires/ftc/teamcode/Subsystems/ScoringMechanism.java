package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.Optional;
import java.util.OptionalDouble;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.BackdropProcessor.PixelColor;


public class ScoringMechanism extends SubsystemBase {
    //Outputs
    private final Motor elevatorMotor;
    private final Servo rightServo;
    private final Servo leftServo;
    private final CRServo handoff;

    //Inputs
    private final DigitalChannel magSwitch;
    private final ColorSensor colorSensor;
    private final Motor.Encoder armEncoder;

    //CONSTANTS
    private static class ArmConstants {
        public static final double ARM_LENGTH = 13.75; //inches
        public static final double ARM_MIN_ANGLE = 0.0; //Radians
        public static final double ARM_MAX_ANGLE = PI; //Radians TODO
        public static final double ARM_ANGLE_TO_POSE = 1.0 / ARM_MAX_ANGLE;//TODO
    }
    private static class ElevatorConstants {
        public static final double ELEVATOR_MAX_DISTANCE = 13.5; //Inches
        public static final double ELEVATOR_SLOPE = 1.7320508;//Math.sqrt(3); //30 - 60 -90 triangle;
        public static final double ELEVATOR_DISTANCE_PER_PULSE = 5.5 / 325.0;
    }
    private static class PositionLookup{
        //The points between wich the arm can't move when the elevator is not extended
        public static final double frontDeadZone = 0.05;
        public static final double backDeadZone = 0.35;

        public static final double[] travelFront = {ElevatorConstants.ELEVATOR_MAX_DISTANCE/ElevatorConstants.ELEVATOR_DISTANCE_PER_PULSE,0.05};
        public static final double[] travelBack = {ElevatorConstants.ELEVATOR_MAX_DISTANCE/ElevatorConstants.ELEVATOR_DISTANCE_PER_PULSE,0.4};
        public static final double[][] pixelLevel = {
                {0.0,0.05}, //Home Position
        };
    }



    //Variables
    private double armAngle = 0.0;
    private double elevatorLength = 0.0;
    private final PIDFController elevatorController;
    private double handoffPower = 0.0;


    private final Telemetry telemetry;


    public ScoringMechanism(HardwareMap hardwareMap, Telemetry telemetry){
        //Output Initialization
        elevatorMotor = new Motor(hardwareMap,"Elevator");
        rightServo = hardwareMap.get(Servo.class,"Arm 1");
        leftServo = hardwareMap.get(Servo.class,"Arm 2");
        handoff = hardwareMap.get(CRServo.class,"Handoff");

        //Sensor Initialization
        magSwitch = hardwareMap.get(DigitalChannel.class,"Elevator Zero");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class,"Color Sensor");
        colorSensor.enableLed(false);

        armEncoder = new Motor(hardwareMap,"Back Right/Arm Encoder").encoder;

        this.telemetry = telemetry;

        //Arm Config
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);

        //Elevator Config
        elevatorMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        elevatorMotor.setInverted(true);


        elevatorController = new PIDFController(0.005,0.0,0.00,0.0);
        elevatorController.setTolerance(15);
        elevatorController.setSetPoint(0.0);

        resetElevatorEncoder();
    }

    @Override
    public void periodic(){

        handoff.setDirection(DcMotorSimple.Direction.FORWARD);
        handoff.setPower(handoffPower);


        Optional<double[]> state = getStateCorrections(elevatorLength,armAngle);
        if(!state.isPresent()) return;

        elevatorController.setSetPoint(state.get()[0]);

        //Elevator Movement
        double elevatorPower = elevatorController.calculate(elevatorMotor.getCurrentPosition());

        //Checks if reaching lower bound of elevator
        if(!magSwitch.getState() && elevatorPower < 0.0){
            elevatorPower = -0.2;
        }

        //Checks if reaching upper bound of elevator
        if((elevatorMotor.getCurrentPosition() * ElevatorConstants.ELEVATOR_DISTANCE_PER_PULSE)  > ElevatorConstants.ELEVATOR_MAX_DISTANCE && elevatorPower > 0.0) {
            elevatorPower = 0.0;
        }

        elevatorMotor.set(elevatorPower);

        //Arm Movement
        leftServo.setPosition(state.get()[1]);
        rightServo.setPosition(state.get()[1]);

        //Telemetry
        telemetry.addLine(String.format("Ticks:%d",elevatorMotor.encoder.getPosition()));
        telemetry.addLine(String.format("ET:%4.2f",elevatorController.getSetPoint()));
        telemetry.addLine();
        telemetry.addLine(String.format("AP:%4.2f",leftServo.getPosition()));



    }

    //Setters
    public void setTargetState(double elevatorLength, double armAngle){
        this.armAngle = armAngle;
        this.elevatorLength = elevatorLength;
    }
    public void setPixelTarget(int level){
        if(level>PositionLookup.pixelLevel.length) return;

        double[] state = PositionLookup.pixelLevel[level];
        elevatorLength = state[0];
        armAngle = state[1];
    }
    public void setHandoff(double power){
        handoffPower = power;
    }

    public void resetElevatorEncoder(){

        elevatorMotor.resetEncoder();
    }


    //Getters
    public double getElevatorLength(){
        return elevatorMotor.getCurrentPosition() * ElevatorConstants.ELEVATOR_DISTANCE_PER_PULSE;
    }
    public  double getArmAngle(){
        //TODO:Return actual arm angle
        return 0.0;
    }

    public PixelColor getPixelColor(){
        //TODO: return the correct colors for the pixels
        return PixelColor.None;
    }

    private OptionalDouble getServoPoseFromAngle(double angle){
        if(angle > ArmConstants.ARM_MAX_ANGLE  || angle < ArmConstants.ARM_MIN_ANGLE){
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(
                angle * ArmConstants.ARM_ANGLE_TO_POSE
        );
    }

    private Optional<double[]> getStateCorrections(double elevatorLength, double armAngle){
        OptionalDouble tempPose = getServoPoseFromAngle(armAngle);

        //Checks if there is no servo position for the given angle
        double armPosition;
        if(!tempPose.isPresent()) return Optional.empty();
        else armPosition = tempPose.getAsDouble();


        //Checks from going from the front to back
//        if(
//                leftServo.getPosition() < PositionLookup.travelBack[1]
//                && armPosition > PositionLookup.travelFront[1]
//                && elevatorMotor.getCurrentPosition() < PositionLookup.travelBack[0]
//            ) {
//                elevatorLength = PositionLookup.travelBack[0];
//                armPosition = PositionLookup.travelBack[1];
//            }
//
//        //Checks for going from the back to front
//        if(
//                leftServo.getPosition() < PositionLookup.travelFront[1]
//                && armPosition > PositionLookup.travelBack[1]
//                && elevatorMotor.getCurrentPosition() < PositionLookup.travelFront[0]
//        ) {
//            elevatorLength = PositionLookup.travelFront[0];
//            armPosition = PositionLookup.travelFront[1];
//        }

        return Optional.of(new double[]{
                elevatorLength,
                armPosition
        });
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

        double a = 1 + Math.pow(ElevatorConstants.ELEVATOR_SLOPE,2);
        double b = -2 * (distance + (ElevatorConstants.ELEVATOR_SLOPE*height));
        double c = -(Math.pow(ArmConstants.ARM_LENGTH,2) - Math.pow(height,2) - Math.pow(distance,2));
        double[] xCoordinates =  solveQuadratic(a,b,c);


        ArrayList<double[]> possibleStates = new ArrayList<>();

        //Adds
        getStateFromJointCordinate(xCoordinates[0],distance).ifPresent(possibleStates::add);
        getStateFromJointCordinate(xCoordinates[1],distance).ifPresent(possibleStates::add);



        return getBestStatePossibility(possibleStates);
    }
    private Optional<double[]> getStateFromJointCordinate(double x, double distance){
        //Get using pythagorean theorem
        double elevatorDistance = Math.sqrt(Math.pow(x,2) + Math.pow(x * ElevatorConstants.ELEVATOR_SLOPE,2));

        // cos(armAngle) = (x - targetX) / ARM_LENGTH
        double armAngle = Math.acos( (x - distance) / ArmConstants.ARM_LENGTH); //TODO: Understand this better

        if(Double.isNaN(elevatorDistance) || Double.isNaN(armAngle)) return Optional.empty();

        return Optional.of(new double[] {
                elevatorDistance,
                armAngle,
        });
    }

    public boolean setTargetEndPoint(double distance, double height){
        Optional<double[]> stateOptional =  getStateFromTargetPoint(distance,height);


        if(!stateOptional.isPresent()) return false;

        double[] state = stateOptional.get();

        elevatorController.setSetPoint(state[0]);
        armAngle = state[1];

        return true;
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
            if( state[0] < 0 || state[0] > ElevatorConstants.ELEVATOR_MAX_DISTANCE) continue;

            if(state[1] < ArmConstants.ARM_MIN_ANGLE || state[1] > ArmConstants.ARM_MAX_ANGLE) continue;

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
