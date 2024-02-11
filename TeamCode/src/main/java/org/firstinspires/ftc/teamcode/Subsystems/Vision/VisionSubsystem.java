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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

public class VisionSubsystem extends SubsystemBase {

    private final WebcamName frontCamera;
//    private final WebcamName backCamera;
    private final OpenCvCamera cameraOpenCV;

//    private final VisionPortal visionPortal;

    private final AprilTagProcessor aprilTagProcessor;
    private final ExtenderPipeline extenderPipeline;
    private TfodProcessor teamPropProcessor;
    private BackdropProcessor backdropProcessor = new BackdropProcessor();

    private boolean isFrontCameraOpen = false;

    private final Telemetry telemetry;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry){

        //Initalize Cameras
        frontCamera = hardwareMap.get(WebcamName.class,"Front Camera");
//        backCamera = hardwareMap.get(WebcamName.class,"Back Camera");

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawCubeProjection(true)
                .build();

        extenderPipeline = new ExtenderPipeline();

//        visionPortal = new VisionPortal.Builder()
//                .setCamera(frontCamera)
//                .enableLiveView(true)
////                .setCamera(backCamera)
////                .addProcessor(aprilTagProcessor)
////                .addProcessor(backdropProcessor)
////                .addProcessor(teamPropProcessor)
//                .build();

//        visionPortal.resumeStreaming();

        //Custom OpenCV pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cameraOpenCV = OpenCvCameraFactory.getInstance().createWebcam(frontCamera, cameraMonitorViewId);

        cameraOpenCV.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                telemetry.addLine("Camera Working");
                isFrontCameraOpen = true;
                setFrontCameraEnabled(true);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("OpenCV Camera Error");
            }
        });

        cameraOpenCV.setPipeline(extenderPipeline);


        this.telemetry = telemetry;

    }

    @Override
    public void periodic(){
//        visionPortal.resumeStreaming();
        telemetry.addLine(String.format("Dist: %4.2f",extenderPipeline.getDistanceFromFrame()));
        telemetry.addLine(String.format("Pix: %d",extenderPipeline.getRawPixelsFromFrame()));
    }

    public void setFrontCameraEnabled(boolean enabled){
        if(enabled && isFrontCameraOpen){
            cameraOpenCV.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
        }else{
            cameraOpenCV.stopStreaming();
        }
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

    public double getExtenderDistance(){
        extenderPipeline.getDistanceFromFrame();

        //TODO:Math to calculate the distance
        return 0.0;
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


}
