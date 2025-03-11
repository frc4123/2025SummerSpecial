package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Telemetry {
    //private final double maxSpeed; // Retain maxSpeed for potential use
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

    /* Robot pose for field positioning */
    private final NetworkTable poseTable = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = poseTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = poseTable.getStringTopic(".type").publish();

    private final double[] poseArray = new double[3];

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        //this.maxSpeed = maxSpeed;
        fieldTypePub.set("Field2d");
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);

        /* Telemeterize the pose to a Field2d */
        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(poseArray);

        /* Example: Normalize module speeds for visualization (if needed) */
        // for (int i = 0; i < state.ModuleStates.length; i++) {
        //     double normalizedSpeed = state.ModuleStates[i].speedMetersPerSecond / maxSpeed;
        //     // Publish normalized speed or use it for visualization
        // }
    }
}
