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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private Backdrop backdropState;

    private final Telemetry telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry){

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawCubeProjection(true)
                .build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Back Camera"))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();

        visionPortal.resumeStreaming();

        this.telemetry = telemetry;

    }

    @Override
    public void periodic(){
//        visionPortal.resumeStreaming();

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

    public Backdrop getBackdropState(){
        return this.backdropState;
    }


}
