package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ObjectMapper;

public class Vision extends SubsystemBase{

    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    public Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public AprilTagFieldLayout aprilTagFieldLayout = loadAprilTagFieldLayout("/edu/wpi/first/apriltag/2025-reefscape.json");
    public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    
    static final Set<Integer> redTargets = new HashSet<>(Arrays.asList(1,2,3,4,5,6,7,8,9,10,11));
    static final Set<Integer> blueTargets = new HashSet<>(Arrays.asList(12,13,14,15,16,17,18,19,20,21,22));

    static final Set<Integer> blueReef = new HashSet<>(Arrays.asList(17,18,19,20,21,22));
    static final Set<Integer> redReef = new HashSet<>(Arrays.asList(6,7,8,9,10,11));

    static final Set<Integer> blueBarge = new HashSet<>(Arrays.asList(4,14));
    static final Set<Integer> redBarge = new HashSet<>(Arrays.asList(5,15));

    static final Set<Integer> blueCoralStation = new HashSet<>(Arrays.asList(12,13));
    static final Set<Integer> redCoralStation = new HashSet<>(Arrays.asList(1,2));

    static final int blueProcessor = 16;
    static final int redProcessor = 3;

    public enum DetectedAlliance {RED, BLUE, NONE};


    public Vision(RobotState state) {
    }

    public static AprilTagFieldLayout loadAprilTagFieldLayout(String resourceFile) { 
        try (InputStream is = Vision.class.getResourceAsStream(resourceFile); 
        InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) { 
            ObjectMapper mapper = new ObjectMapper(); 
            return mapper.readValue(isr, AprilTagFieldLayout.class); 
        } catch (IOException e) { 
            throw new UncheckedIOException(e); 
        } 
    }

    public Pose3d get3dPose() {
        var result = getCamResult(); 
        if (result.hasTargets()) { 
            PhotonTrackedTarget target = result.getBestTarget(); 
            Optional<Pose3d> optionalPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()); 


            Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), optionalPose.get(), robotToCam);
            return cameraRobotPose; 
        } else { 
            return null; 
        }
    }

    public DetectedAlliance getAllianceStatus() {
        var result = getCamResult(); 
        List<PhotonTrackedTarget> targets = result.getTargets(); 
        var redTargetCount = 0;
        var blueTargetCount = 0;

        for (PhotonTrackedTarget target : targets) {
            if (redTargets.contains(target.getFiducialId())) {
                redTargetCount += 1;
            }
            if (blueTargets.contains(target.getFiducialId())) {
                blueTargetCount += 1;
            }
        }

        if (redTargetCount > blueTargetCount && redTargetCount >= 1) {
            return DetectedAlliance.RED;
        } else if (blueTargetCount > redTargetCount && blueTargetCount >= 1) {
            return DetectedAlliance.BLUE;
        } else {
            return DetectedAlliance.NONE;
        }
    }

    public PhotonPipelineResult getCamResult() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults(); 
        if (results.isEmpty()) { 
            return new PhotonPipelineResult(); 
        } 
        return results.get(results.size() - 1); 
    }

    public boolean hasTarget() {
        var result = getCamResult(); 
        return result.hasTargets(); 
    }

    public double getCamTimeStamp() {
        var imageCaptureTime = getCamResult().getTimestampSeconds(); 
        return imageCaptureTime; 
    }
           
    @Override
    public void periodic() {
    }


}
