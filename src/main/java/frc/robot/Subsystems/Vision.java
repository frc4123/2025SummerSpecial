package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.generated.TunerConstants;

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

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class Vision extends SubsystemBase{

    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    public Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public AprilTagFieldLayout aprilTagFieldLayout = loadAprilTagFieldLayout("/fields/Reefscape2025.json");
    //public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);    
    public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    
    static final Set<Integer> redTargets = new HashSet<>(Arrays.asList(1,2,3,4,5,6,7,8,9,10,11));
    static final Set<Integer> blueTargets = new HashSet<>(Arrays.asList(12,13,14,15,16,17,18,19,20,21,22));

    static final Set<Integer> blueReef = new HashSet<>(Arrays.asList(17,18,19,20,21,22));
    static final Set<Integer> redReef = new HashSet<>(Arrays.asList(6,7,8,9,10,11));

    static final Set<Integer> blueBarge = new HashSet<>(Arrays.asList(4,14));
    static final Set<Integer> redBarge = new HashSet<>(Arrays.asList(5,15));

    static final Set<Integer> blueCoralStation = new HashSet<>(Arrays.asList(12,13));
    static final Set<Integer> redCoralStation = new HashSet<>(Arrays.asList(1,2));

    static final Set<Integer> blueProcessor = new HashSet<>(Arrays.asList(16));
    static final Set<Integer> redProcessor = new HashSet<>(Arrays.asList(3));

    public enum DetectedAlliance {RED, BLUE, NONE};

    private List<PhotonPipelineResult> currentResultList;
    private PhotonPipelineResult currentResult;

    private final CommandSwerveDrivetrain drivetrain; 

    public Vision(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
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
    

    public DetectedAlliance getAllianceStatus() {
        var result = currentResult;
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
        } else return DetectedAlliance.NONE;
    }

    public Pose3d get3dPose() {
        var result = currentResult; 
        if (result != null) { 
            PhotonTrackedTarget target = result.getBestTarget(); 
            Optional<Pose3d> optionalPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()); 

            Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), optionalPose.get(), robotToCam);
            return cameraRobotPose; 
        } else return null; 
    }
    
    public Pose2d get2dPose() {
        if (get3dPose() != null) {
            Pose2d convertedPose2d = get3dPose().toPose2d();
            return convertedPose2d;
        } else return null;
    }

    public boolean hasTarget() {
        if (currentResult != null){
            var result = currentResult.hasTargets();
            return result; 
        } else return false;
    }

    public double getCamTimeStamp() {
            double imageCaptureTime = currentResult.getTimestampSeconds(); 
            return imageCaptureTime; 
    }

    // public PhotonTrackedTarget getBestTarget() {
    //     if (hasTarget()){
    //         PhotonTrackedTarget target = getCamResult().
    //         return target;
    //     } else {
    //         return null;
    //     }
        
    // }

    public int getBestAprilTagId(){
        if(hasTarget() && currentResult != null){
            return currentResult.getBestTarget().getFiducialId();
        } else return 0;
        
    }

    public Rotation2d getAngleToAprilTag() {
        if (hasTarget() && currentResult != null){
            double yaw = currentResult.getBestTarget().getYaw(); 
            Rotation2d cameraYaw = Rotation2d.fromDegrees(yaw);
            Rotation2d robotToCamera = new Rotation2d(0); // Replace with your camera's mounting ang
            return cameraYaw.plus(robotToCamera);
        } else {
            return null;
        }
        
    }

    public Command driveToPose(){
        var targetPose2d = getTargetPose2d();
            if(targetPose2d != null){
                PathConstraints pathConstraints = new PathConstraints( 
                    TunerConstants.kSpeedAt12Volts, TunerConstants.kLinearAcceleration,
                    TunerConstants.kAngularVelocity, TunerConstants.kAngularAcceleration
                );
                return AutoBuilder.pathfindToPose(
                    getTargetPose2d(),
                    pathConstraints,
                    0.0 
                );
            } else return Commands.none();
    }
        
    public Rotation2d getDegreesToGamePiece() {
        // Axis are flipped due to FieldCentric M
        if (hasTarget() && currentResult != null) {
            int id = getBestAprilTagId();
            if (DriverStation.getAlliance().get() == Alliance.Red){
                switch(getClosestGamePiece(id)){
                    case "Red Reef":
                        switch (id) {
                            case 6: return Rotation2d.fromDegrees(300);
                            case 7: return Rotation2d.fromDegrees(0);
                            case 8: return Rotation2d.fromDegrees(60);
                            case 9: return Rotation2d.fromDegrees(120);
                            case 10: return Rotation2d.fromDegrees(180);
                            case 11: return Rotation2d.fromDegrees(240);
                        }
                        break;
                    case "Red Barge":
                        switch (id) {
                            case 15: return Rotation2d.fromDegrees(180); // 
                            case 5: return Rotation2d.fromDegrees(0); // 
                        }
                        break;

                    case "Red Coral Station":
                        switch (id) {
                            case 2: return Rotation2d.fromDegrees(245); // 
                            case 1: return Rotation2d.fromDegrees(125); // 
                        }
                        break;
                    case "Red Processor":
                        return Rotation2d.fromDegrees(270);
                    default:
                        return null; 
                    }
            } else {
                switch (getClosestGamePiece(id)) {      
                    case "Blue Reef":
                        switch (id) {
                            case 17: return Rotation2d.fromDegrees(60); // 
                            case 18: return Rotation2d.fromDegrees(0); // 
                            case 19: return Rotation2d.fromDegrees(300); // 
                            case 20: return Rotation2d.fromDegrees(240); //
                            case 21: return Rotation2d.fromDegrees(180); //
                            case 22: return Rotation2d.fromDegrees(120); // 
                        }
                        break;
                    case "Blue Barge":
                        switch (id) {
                            case 14: return Rotation2d.fromDegrees(0); // 
                            case 4: return Rotation2d.fromDegrees(180); // 
                        }
                        break;
                    case "Blue Coral Station":
                        switch (id) {
                            case 12: return Rotation2d.fromDegrees(245); // 
                            case 13: return Rotation2d.fromDegrees(125); // 
                        }
                        break;
                    case "Blue Processor":
                        return Rotation2d.fromDegrees(270);
                    default: return null;
                    }
            }  
        }
        return null; 
    }
    
    
    public String getClosestGamePiece(int id) {
        if(hasTarget() && currentResult != null){
            if (blueReef.contains(id)){
                return "Blue Reef";
            } else if(redReef.contains(id)){
                return "Red Reef";
            } else if(blueBarge.contains(id)){
                return "Blue Barge";
            } else if(redBarge.contains(id)){
                return "Red Barge";
            } else if(blueCoralStation.contains(id)){
                return "Blue Coral Station";
            } else if(redCoralStation.contains(id)){
                return "Red Coral Station";
            } else if(blueProcessor.contains(id)){
                return "Blue Processor";
            } else if(redProcessor.contains(id)){
                return "Red Processor";
            } else return "Detected ID but not game piece, check code";
        } else return "none";
    }

    public Pose2d getTargetPose2d(){
        // Axis are flipped due to FieldCentric M
        if (hasTarget() && currentResult != null) {
            int id = getBestAprilTagId();
            if (DriverStation.getAlliance().get() == Alliance.Red){
                switch(getClosestGamePiece(id)){
                    case "Red Reef":
                        switch (id) {
                            case 6: return new Pose2d(0,0, Rotation2d.fromDegrees(300)); // L 13.581 2.788 | R 13.887 | 2.941
                            case 7: return new Pose2d(0,0, Rotation2d.fromDegrees(0)); // L 14.393 3.851 | R 14.393 4.182
                            case 8: return new Pose2d(0,0, Rotation2d.fromDegrees(60)); // L 13.882 5.097 | R 13.595 5.259
                            case 9: return new Pose2d(0,0, Rotation2d.fromDegrees(120)); // L 12.555 5.284 | R 12.250 5.113
                            case 10: return new Pose2d(0,0, Rotation2d.fromDegrees(180)); // L 11.71 4.18 | R 11.700 3.857
                            case 11: return new Pose2d(0,0, Rotation2d.fromDegrees(240)); // L 12.266 2.951 | R 12.536 2.788
                        }
                        break;
                    case "Red Barge":
                        switch (id) {
                            case 15: return new Pose2d(0,0, Rotation2d.fromDegrees(180)); // X-> 7.711 Y -> use current state
                            case 5: return new Pose2d(0,0, Rotation2d.fromDegrees(0)); // X -> 9.842 Y -> use current state
                        }
                        break;

                    case "Red Coral Station":
                        switch (id) {
                            case 2: return new Pose2d(0,0, Rotation2d.fromDegrees(245)); // 16.258 7.049
                            case 1: return new Pose2d(0,0, Rotation2d.fromDegrees(125)); // 16.305 1.017
                        }
                        break;
                    case "Red Processor":
                        return new Pose2d(0,0, Rotation2d.fromDegrees(270)); // 11.524 7.471
                    default:
                        return null; 
                    }
            } else {
                switch (getClosestGamePiece(id)) {      
                    case "Blue Reef":
                        switch (id) {
                            case 17: return new Pose2d(3.687,2.922, Rotation2d.fromDegrees(60)); // L 3.687 2.922 | R 3.951 2.771
                            case 18: return new Pose2d(3.129,4.179, Rotation2d.fromDegrees(0)); // L 3.129 4.179 | R 3.129 3.850
                            case 19: return new Pose2d(3.96,5.271, Rotation2d.fromDegrees(300)); // L 3.96 5.271 | R 3.672 5.12
                            case 20: return new Pose2d(5.298,5.11, Rotation2d.fromDegrees(240)); // L 5.298 5.110 | R 5.082 5.273
                            case 21: return new Pose2d(5.839,3.853, Rotation2d.fromDegrees(180)); // L 5.839 3.853 | R 5.839 4.168
                            case 22: return new Pose2d(5.013,2.788, Rotation2d.fromDegrees(120)); // L 5.013 2.788 | R 5.298 2.950
                        }
                        break;
                    case "Blue Barge":
                        switch (id) {
                            case 14: return new Pose2d(0,0,  Rotation2d.fromDegrees(0)); // 7.652 6.170
                            case 4: return new Pose2d(0,0, Rotation2d.fromDegrees(180)); // 9.865 6.170
                        }
                        break;
                    case "Blue Coral Station":
                        switch (id) {
                            case 12: return new Pose2d(0,0, Rotation2d.fromDegrees(245)); // 1.416 0.879
                            case 13: return new Pose2d(0,0, Rotation2d.fromDegrees(125)); // 1.586 7.262
                        }
                        break;
                    case "Blue Processor":
                        return new Pose2d(0,0, Rotation2d.fromDegrees(270)); // 6.371 0.645
                    default: return null;
                    }
            }  
        }
        return null;
    }
           
    @Override
    public void periodic() {
        currentResultList = camera.getAllUnreadResults();
        for (int i = currentResultList.size() - 1; i >= 0; i--) {
            PhotonPipelineResult result = currentResultList.get(i);
            if (result.hasTargets()) {
                currentResult = result;
                break;
            } else currentResult = null;
        }

        if (hasTarget()){
            drivetrain.addVisionMeasurement(get2dPose(), getCamTimeStamp());
        }

        SmartDashboard.putNumber("Pose X", drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber("Pose Y", drivetrain.getState().Pose.getY());
        SmartDashboard.putNumber("Pose Rotation (Degrees)", drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Focused April Tag: ", getBestAprilTagId());
        SmartDashboard.putString("Game Piece in Focus: ", getClosestGamePiece(getBestAprilTagId()));
    }
}
