package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    private final CommandSwerveDrivetrain drivetrain;

    // Standard deviations - more conservative initial values
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);  // Higher uncertainty for single tags
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.8, 0.8, 1.5);
    
    // Alliance awareness
    private Alliance currentAlliance = Alliance.Blue;
    
    // Game piece tracking
    private Rotation2d lastGamePieceAngle = new Rotation2d();
    
    // Velocity-based trust factors
    private double lastUpdateTime = 0;
    private double lastVelocityMagnitude = 0;
    
    // Camera selection
    private enum ActiveCamera { NONE, LEFT, RIGHT }
    private ActiveCamera lastUsedCamera = ActiveCamera.NONE;

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

        // Estimators
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
    }

    @Override
    public void periodic() {
        // Update alliance if changed
        updateAlliance();
        
        // Process cameras and select the best result
        processCameras();
        
        // Update game piece tracking
        updateGamePieceTracking();
    }

    private void updateAlliance() {
        if (DriverStation.isDSAttached()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                currentAlliance = alliance;
            });
        }
    }

    private void processCameras() {
        PhotonPipelineResult rightResult = getLatestValidResult(rightCamera);
        PhotonPipelineResult leftResult = getLatestValidResult(leftCamera);
        
        // Score each camera result
        double rightScore = scoreCameraResult(rightResult, rightCamera);
        double leftScore = scoreCameraResult(leftResult, leftCamera);
        
        // Only use the better result if it meets minimum quality threshold
        if (rightScore > leftScore && rightScore > 0.5) {
            processSingleCamera(rightResult, rightEstimator, true);
            lastUsedCamera = ActiveCamera.RIGHT;
        } else if (leftScore > 0.5) {
            processSingleCamera(leftResult, leftEstimator, false);
            lastUsedCamera = ActiveCamera.LEFT;
        } else {
            lastUsedCamera = ActiveCamera.NONE;
        }
    }

    private PhotonPipelineResult getLatestValidResult(PhotonCamera camera) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets() || result.getTimestampSeconds() == lastUpdateTime) {
            return null;
        }
        return result;
    }

    private double scoreCameraResult(PhotonPipelineResult result, PhotonCamera camera) {
        if (result == null || !result.hasTargets()) return 0;
        
        // Get best target
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget == null) return 0;
        
        // Calculate score based on multiple factors
        double score = 0;
        
        // 1. Number of tags (higher is better)
        int numTags = result.targets.size();
        score += numTags * 0.3;
        
        // 2. Target ambiguity (lower is better)
        double ambiguity = bestTarget.getPoseAmbiguity();
        score += (1 - ambiguity) * 0.4;
        
        // 3. Distance to tag (closer is better)
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
        if (tagPose.isPresent()) {
            double distance = tagPose.get().toPose2d().getTranslation()
                .getDistance(drivetrain.getState().Pose.getTranslation());
            score += (1 - Math.min(distance / 10, 1)) * 0.3;  // Normalize distance to 0-10m
        }
        
        return score;
    }

    private void processSingleCamera(PhotonPipelineResult result, PhotonPoseEstimator estimator, boolean isRightCamera) {
        if (result == null || !result.hasTargets() || !isTagReef(result)) return;
        
        estimator.setReferencePose(drivetrain.getState().Pose);
        Optional<EstimatedRobotPose> poseOptional = estimator.update(result);
        
        if (poseOptional.isPresent()) {
            EstimatedRobotPose est = poseOptional.get();
            
            // Calculate dynamic standard deviations
            Matrix<N3, N1> stdDevs = calculateDynamicStdDevs(est, result);
            
            // Apply vision measurement
            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                Utils.fpgaToCurrentTime(est.timestampSeconds),
                stdDevs
            );
            
            lastUpdateTime = est.timestampSeconds;
        }
    }

    private boolean isTagReef(PhotonPipelineResult result) {
        if (!result.hasTargets()) return false;
        
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget == null) return false;
        
        int tagId = bestTarget.getFiducialId();
        
        // Blue alliance targets
        if (currentAlliance == Alliance.Blue) {
            return Set.of(17,18,19,20,21,22).contains(tagId);
        } 
        // Red alliance targets
        else {
            return Set.of(6,7,8,9,10,11).contains(tagId);
        }
    }

    private Matrix<N3, N1> calculateDynamicStdDevs(EstimatedRobotPose est, PhotonPipelineResult result) {
        // Base standard deviations based on number of tags
        Matrix<N3, N1> baseDevs = result.targets.size() >= 2 ? multiTagStdDevs : singleTagStdDevs;
        
        // Calculate average distance to tags
        double totalDistance = 0;
        int validTags = 0;
        
        for (PhotonTrackedTarget target : result.targets) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            
            validTags++;
            totalDistance += tagPose.get().toPose2d().getTranslation()
                .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }
        
        if (validTags == 0) return baseDevs.times(10);  // High uncertainty if no valid tags
        
        double avgDistance = totalDistance / validTags;
        
        // Distance factor (quadratic increase with distance)
        double distanceFactor = 1 + (avgDistance * avgDistance / 10);
        
        // Ambiguity factor (from best target)
        double ambiguityFactor = 1 + (result.getBestTarget().getPoseAmbiguity() * 3);
        
        // Velocity factor (based on robot movement)
        ChassisSpeeds speeds = drivetrain.getState().Speeds; // Get chassis speeds
        double currentVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); // Calculate magnitude
    
        double velocityFactor = 1 + (Math.abs(currentVelocity) / 3);  // More uncertainty when moving fast
        
        // Combine all factors
        return baseDevs.times(distanceFactor * ambiguityFactor * velocityFactor);
    }

    private void updateGamePieceTracking() {
        int closestTag = findClosestAprilTag();
        lastGamePieceAngle = calculateGamePieceAngle(closestTag);
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
        if (currentAlliance == Alliance.Red && Set.of(6,7,8,9,10,11).contains(tagId)) {
            return Rotation2d.fromDegrees((tagId - 7) * 60);
        }
        // Blue targets
        if (currentAlliance == Alliance.Blue && Set.of(17,18,19,20,21,22).contains(tagId)) {
            return Rotation2d.fromDegrees((tagId - 18) * -60);
        }
        // Default case
        return new Rotation2d();
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
    public boolean hasActiveVisionMeasurement() { return lastUsedCamera != ActiveCamera.NONE; }
    public ActiveCamera getActiveCamera() { return lastUsedCamera; }
}