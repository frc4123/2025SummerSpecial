package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.*;
import org.photonvision.EstimatedRobotPose;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.fasterxml.jackson.databind.ObjectMapper;

public class Vision extends SubsystemBase {
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera rightCamera;
    private final PhotonCamera leftCamera;
    private final PhotonPoseEstimator rightEstimator;
    private final PhotonPoseEstimator leftEstimator;
    private final Transform3d robotToRightCam;
    private final Transform3d robotToLeftCam;
    
    // Standard deviations (tune these based on camera characteristics)
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(3, 3, 3);
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
    // Alliance awareness
    private int blueInversionFactor = 0;
    private int redInversionFactor = 0;
    
    // Game piece tracking
    private Rotation2d lastGamePieceAngle = new Rotation2d();
    private Pose2d lastTargetPoseRight = new Pose2d();
    private Pose2d lastTargetPoseLeft = new Pose2d();
    
    // NetworkTables publishers
    private final StructPublisher<Pose3d> rightCamPosePublisher;
    private final StructPublisher<Pose3d> leftCamPosePublisher;
    private final StructPublisher<Transform3d> rightCamTargetTransformPublisher;
    private final StructPublisher<Transform3d> leftCamTargetTransformPublisher;
    
    private final CommandSwerveDrivetrain drivetrain;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.aprilTagFieldLayout = loadAprilTagFieldLayout("/fields/Reefscape2025.json");

        // Camera configuration
        rightCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Right");
        leftCamera = new PhotonCamera("Arducam_OV9281_USB_Camera_Left");
        
        // Camera transforms
        robotToRightCam = new Transform3d(
            new Translation3d(
                Constants.VisionConstants.rightX,
                Constants.VisionConstants.rightY,
                Constants.VisionConstants.rightZ),
            new Rotation3d(
               Constants.VisionConstants.rightRoll,
                Constants.VisionConstants.rightPitch,
                Constants.VisionConstants.rightYaw)
        );

        robotToLeftCam = new Transform3d(
            new Translation3d(
                Constants.VisionConstants.leftX,
                Constants.VisionConstants.leftY,
                Constants.VisionConstants.leftZ),
            new Rotation3d(
                Constants.VisionConstants.leftRoll,
                Constants.VisionConstants.leftPitch,
                Constants.VisionConstants.leftYaw)
        );

        // Right camera estimator (new 2025 syntax)
        rightEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToRightCam
        );

        leftEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToLeftCam
        );
        leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Configure alliance inversion
        if (DriverStation.isDSAttached()) {
            blueInversionFactor = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 180;
            redInversionFactor = 180 - blueInversionFactor;
        }

        // Initialize NetworkTables publishers
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        rightCamPosePublisher = inst.getStructTopic("/Vision/RightCameraPose", Pose3d.struct).publish();
        leftCamPosePublisher = inst.getStructTopic("/Vision/LeftCameraPose", Pose3d.struct).publish();
        rightCamTargetTransformPublisher = inst.getStructTopic("/Vision/RightCamTargetTransform", Transform3d.struct).publish();
        leftCamTargetTransformPublisher = inst.getStructTopic("/Vision/LeftCamTargetTransform", Transform3d.struct).publish();
    }

    @Override
    public void periodic() {
        // Process both cameras
        processCamera(rightCamera, rightEstimator);
        processCamera(leftCamera, leftEstimator);
        
        // Update game piece tracking
        updateGamePieceTracking();
        
        // Publish camera poses
        publishCameraPoses();
    }

    ////////
    public static PhotonPipelineResult getLatestResults(PhotonCamera camera) {
        List<PhotonPipelineResult> currentResultList = camera.getAllUnreadResults();
        
        // Search from newest (end of list) to oldest (start of list)
        for (int i = currentResultList.size() - 1; i >= 0; i--) {
            PhotonPipelineResult result = currentResultList.get(i);
            if (result.hasTargets()) {
                return result;  // Return first valid result with targets
            }
        }
        return null;  // No valid results found
    }
    ////////

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        PhotonPipelineResult result = getLatestResults(camera);
        if (result == null || !result.hasTargets()) return;
        

        if (isTagReef(result)){
            // Null check first before accessing methods
            
    
            estimator.setReferencePose(drivetrain.getState().Pose);
            Optional<EstimatedRobotPose> poseOptional = estimator.update(result);
        
            if (poseOptional.isPresent()) {
                EstimatedRobotPose est = poseOptional.get();
                Matrix<N3, N1> stdDevs = calculateStdDevs(est, result.getTargets());
            
                drivetrain.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    Utils.fpgaToCurrentTime(est.timestampSeconds),
                    stdDevs
                );
            
                // Publish target transforms
                publishTargetTransform(result.getBestTarget(), camera.equals(rightCamera));
            }
        }
        
    }

    private boolean isTagReef(PhotonPipelineResult result){
        int tagId = result.getBestTarget().getFiducialId();
        if (Set.of(17,18,19,20,21,22).contains(tagId) && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return true;
        } else if (Set.of(6,7,8,9,10,11).contains(tagId) && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return true;
        }
        return false;
    }

    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est, List<PhotonTrackedTarget> targets) {
        int numTags = 0;
        double totalDistance = 0;

        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            
            numTags++;
            totalDistance += tagPose.get().toPose2d().getTranslation()
                .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) return singleTagStdDevs;
        
        double avgDistance = totalDistance / numTags;
        Matrix<N3, N1> baseDevs = numTags >= 2 ? multiTagStdDevs : singleTagStdDevs;
        return baseDevs.times(0.2 + (avgDistance * avgDistance / 20));
    }

    private void publishTargetTransform(PhotonTrackedTarget target, boolean isRightCamera) {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) return;

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Transform3d robotToTarget = isRightCamera ? 
            robotToRightCam.plus(cameraToTarget) :
            robotToLeftCam.plus(cameraToTarget);

        if (isRightCamera) {
            rightCamTargetTransformPublisher.set(robotToTarget);
        } else {
            leftCamTargetTransformPublisher.set(robotToTarget);
        }
    }

    private void updateGamePieceTracking() {
        int closestTag = findClosestAprilTag();
        lastGamePieceAngle = calculateGamePieceAngle(closestTag);
        lastTargetPoseLeft = getTargetPose2dLeft(closestTag);
        lastTargetPoseRight = getTargetPose2dRight(closestTag);
    }

    private void publishCameraPoses() {
        Pose3d robotPose = new Pose3d(drivetrain.getState().Pose);
        rightCamPosePublisher.set(robotPose.plus(robotToRightCam));
        leftCamPosePublisher.set(robotPose.plus(robotToLeftCam));
    }

    private int findClosestAprilTag() {
        Pose2d robotPose = drivetrain.getState().Pose;
        return aprilTagFieldLayout.getTags().stream()
            .min(Comparator.comparingDouble(tag -> 
                tag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation())))
            .map(tag -> tag.ID)
            .orElse(-1);
    }

    private Rotation2d calculateGamePieceAngle(int tagId) {
        // Red targets
        if (Set.of(6,7,8,9,10,11).contains(tagId)) {
            return Rotation2d.fromDegrees((tagId - 7) * 60 + redInversionFactor);
        }
        // Blue targets
        if (Set.of(17,18,19,20,21,22).contains(tagId)) {
            return Rotation2d.fromDegrees((tagId - 18) * -60 + blueInversionFactor);
        }
        // Special cases
        switch(tagId) {
            case 15: case 5: return Rotation2d.fromDegrees(180 + redInversionFactor);
            case 14: case 4: return Rotation2d.fromDegrees(blueInversionFactor);
            case 3: case 16: return Rotation2d.fromDegrees(270);
            default: return Rotation2d.fromDegrees(0);
        }
    }

    private Pose2d getTargetPose2dLeft(int tagId) {
        // Implement your specific left target positions
        switch(tagId) {
            case 6: return new Pose2d(13.581, 2.788, Rotation2d.fromDegrees(300 + redInversionFactor));
            case 17: return new Pose2d(3.687, 2.922, Rotation2d.fromDegrees(60 + blueInversionFactor));
            // Add all other cases from your original implementation
            default: return new Pose2d();
        }
    }

    private Pose2d getTargetPose2dRight(int tagId) {
        // Implement your specific right target positions
        switch(tagId) {
            case 6: return new Pose2d(13.887, 2.941, Rotation2d.fromDegrees(300 + redInversionFactor));
            case 17: return new Pose2d(3.951, 2.771, Rotation2d.fromDegrees(60 + blueInversionFactor));
            // Add all other cases from your original implementation
            default: return new Pose2d();
        }
    }

    public static AprilTagFieldLayout loadAprilTagFieldLayout(String resourceFile) {
        try (InputStream is = Vision.class.getResourceAsStream(resourceFile);
             InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) {
            return new ObjectMapper().readValue(isr, AprilTagFieldLayout.class);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    // Getters
    public Rotation2d getLastGamePieceAngle() { return lastGamePieceAngle; }
    public Pose2d getLastTargetPoseLeft() { return lastTargetPoseLeft; }
    public Pose2d getLastTargetPoseRight() { return lastTargetPoseRight; }
}