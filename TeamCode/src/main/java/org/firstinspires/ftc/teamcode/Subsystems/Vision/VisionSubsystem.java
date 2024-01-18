package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

public class VisionSubsystem extends SubsystemBase {

    private final WebcamName frontCamera;
    private final WebcamName backCamera;

    private final VisionPortal visionPortal;

    private final AprilTagProcessor aprilTagProcessor;
    private final TfodProcessor extenderProcessor;
    private TfodProcessor teamPropProcessor;
    private BackdropProcessor backdropProcessor = new BackdropProcessor();


    private final Telemetry telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry){

        //Initalize Cameras
        frontCamera = hardwareMap.get(WebcamName.class,"Front Camera");
        backCamera = hardwareMap.get(WebcamName.class,"Back Camera");

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawCubeProjection(true)
                .build();

        extenderProcessor = new TfodProcessor.Builder()
                //TODO
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .addProcessor(backdropProcessor)
                .addProcessor(extenderProcessor)
                .addProcessor(teamPropProcessor)
                .build();

        visionPortal.resumeStreaming();




        this.telemetry = telemetry;

    }

    @Override
    public void periodic(){
        visionPortal.resumeStreaming();

    }

    //Get Data from processors
    public Optional<Pose2d> getRobotFieldPose(){

        AprilTagDetection detection = null;

        //Finds best April Tag
        for(AprilTagDetection detectionCanidate : aprilTagProcessor.getDetections()) {
            //First Pass
            if(detection == null) {
                detection = detectionCanidate;
                continue;
            }

            if(detectionCanidate.decisionMargin > detection.decisionMargin){
                detection = detectionCanidate;
            }
        }

        if(detection != null){
            double theta = (detection.ftcPose.bearing - detection.ftcPose.yaw);

            Pose2d detectionPose = new Pose2d(
                    new Translation2d(
                            -detection.ftcPose.range * sin(theta),
                            -detection.ftcPose.range * cos(theta)
                    ),
                    new Rotation2d()
            );
            VectorF poseVector = detection.metadata.fieldOrientation.applyToVector(detection.metadata.fieldPosition);

            telemetry.addLine("Vec: " + poseVector.toString());

            Pose2d aprilTagFieldPose = new Pose2d(
                    new Translation2d(
                            poseVector.get(2),
                            poseVector.get(1)
                    ),
                    new Rotation2d(0.0)
            );

            return Optional.of(detectionPose.relativeTo(aprilTagFieldPose));

        }else{
            return Optional.empty();
        }

    }
    public List<Pair<Integer,Pose2d>> getAprilTagsRelativePoses(){
        List<Pair<Integer, Pose2d>> positions = new ArrayList<>();

        for(AprilTagDetection detection : aprilTagProcessor.getDetections()){
            double theta = (detection.ftcPose.bearing - detection.ftcPose.yaw);
            positions.add(
                    new Pair<>(
                            0,
                            new Pose2d(
                                    new Translation2d(
                                            detection.ftcPose.range * sin(theta),
                                            detection.ftcPose.range * cos(theta)
                                    ),
                                    new Rotation2d(
                                            detection.ftcPose.yaw
                                    )
                            )));
        }

        return positions;

    }

    public OptionalDouble getExtenderDistance(){
        List<Recognition> recognitionList  = extenderProcessor.getRecognitions();

        if(recognitionList.isEmpty()) return OptionalDouble.empty();

        Recognition recognition = recognitionList.get(0);

        return OptionalDouble.of(recognition.getBottom() * 20); //TODO: GET ACTUAL PIXELS PER INCH

    }

    public Optional<TeamPropProcessor.PropLocation> getPropLocation(){
        List<Recognition> recognitionList =  teamPropProcessor.getRecognitions();

        if(recognitionList.isEmpty()) return Optional.empty();

        Recognition recognition = recognitionList.get(0);

        float pose = recognition.getRight();

        if(pose < 25){ //Checks if the prop is to the left of the maximum line
            return Optional.of(TeamPropProcessor.PropLocation.Left);
        }

        //Checks if prop is to the right of the minimum line
        if(pose > 75){
            return Optional.of(TeamPropProcessor.PropLocation.Right);
        }

        return Optional.of(TeamPropProcessor.PropLocation.Center);
    }

    public void switchCameraToFront(boolean value){
        if(value) visionPortal.setActiveCamera(frontCamera);
        else  visionPortal.setActiveCamera(backCamera);
    }


}
