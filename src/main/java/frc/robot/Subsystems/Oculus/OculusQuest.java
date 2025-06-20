package frc.robot.subsystems.Oculus;

import static frc.robot.Constants.Oculus.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.Oculus.PoseResetStrategy;
import frc.robot.subsystems.Oculus.io.OculusIO;
import frc.robot.subsystems.Oculus.io.OculusIOInputsAutoLogged;
import frc.robot.commands.utils.DashboardNotifs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Manages communication and pose estimation with a Meta Quest VR headset.
 *
 * <p>This subsystem leverages the Quest's inside-out SLAM tracking system to provide high-frequency
 * (120Hz) robot pose estimation. Key features:
 *
 * <p>- Global SLAM-based localization - Field mapping and persistence - Sub-centimeter tracking
 * precision - High update rate (120Hz) - Drift-free position tracking - Fast relocalization
 *
 * <p>The system operates in phases: 1. Pre-match mapping to capture field features 2. Initial pose
 * acquisition and alignment 3. Continuous pose updates during match 4. Recovery handling if
 * tracking is lost
 */
public class OculusQuest extends SubsystemBase {
  /** Hardware communication interface */
  private final OculusIO io;

  /** Consumer for pose updates from the Oculus */
  private final OculusConsumer oculusConsumer;

  /** Logged inputs from Quest hardware */
  private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

  /** Transform offset applied when using ROBOT_SIDE reset strategy */
  private Transform2d offsetTransform = new Transform2d();

  private boolean questHadTracking = false;

  private final CommandSwerveDrivetrain drivetrain;

  /**
   * Creates a new OculusSubsystem.
   *
   * <p>Initializes communication with Quest hardware and prepares logging systems. The subsystem
   * starts in an uninitialized state requiring pose calibration.
   *
   * @param oculusConsumer Consumer that receives pose updates from the headset
   * @param io Interface for Quest hardware communication
   */
  public OculusQuest(OculusConsumer oculusConsumer, OculusIO io, CommandSwerveDrivetrain drivetrain) {
    this.io = io;
    this.oculusConsumer = oculusConsumer;
    this.drivetrain = drivetrain;
    Logger.recordOutput("Oculus/status", "Initialized");
  }

  /**
   * Updates subsystem state and processes Quest data.
   *
   * <p>Called periodically by the command scheduler. This method: - Updates hardware inputs -
   * Processes new pose data - Handles state transitions - Manages reset operations - Updates
   * logging
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    // Add to Kalman filter
    processPose();

    // Notify if we are disconnected
    if (!inputs.connected) {
      NotificationPresets.Oculus.sendOculusDisconnectedNotification();
    } else {
      NotificationPresets.Oculus.sendOculusReconnectedNotification();
    }

    // Notify for battery levels
    if (inputs.batteryPercent < BATTERY_CRITICAL_PERCENT) {
      NotificationPresets.Oculus.sendOculusBatteryCriticalNotification();
    } else if (inputs.batteryPercent < BATTERY_LOW_PERCENT) {
      NotificationPresets.Oculus.sendOculusBatteryLowNotification();
    }

    // Notify for tracking status
    if (!inputs.isTracking) {
      NotificationPresets.Oculus.sendOculusTrackingLostNotification(inputs.totalTrackingLostEvents);
    } else {
      NotificationPresets.Oculus.sendOculusTrackingRegainedNotification();
    }
  }

  /**
   * Returns the battery percentage of the connected Quest headset.
   *
   * @return Battery percentage (0-100)
   */
  public double getBatteryPercent() {
    return inputs.batteryPercent;
  }

  /**
   * Returns the timestamp of the most recent pose update.
   *
   * @return Timestamp in seconds
   */
  public double getTimestamp() {
    return inputs.timestamp;
  }

  /**
   * Checks if the Quest headset is currently connected.
   *
   * @return True if connected, false otherwise
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Gets the current robot pose as estimated by the Quest headset. This incorporates all transforms
   * and offsets to convert from headset to robot coordinates.
   *
   * @return Field-relative robot pose
   */
  @AutoLogOutput(key = "Oculus/Pose")
  public Pose2d getPose() {
    return getOculusPose().transformBy(ROBOT_TO_OCULUS.inverse()).plus(offsetTransform);
  }

  /**
   * Resets the pose tracking system to a specified position. Must be called only when the robot is
   * disabled to avoid interrupting tracking during a match.
   *
   * @param pose The new reference pose
   */
  public void resetPose(Pose2d pose) {
    if (DriverStation.isEnabled()) {
      Logger.recordOutput(
          "Oculus/Log",
          "resetPose() called while the robot is enabled. This shouldn't happen! Ignoring.");
      return;
    }
    // Transform the pose to the Oculus coordinate system w/ offset
    Pose2d oculusSidePose = pose.plus(ROBOT_TO_OCULUS);

    if (POSE_RESET_STRATEGY.equals(PoseResetStrategy.ROBOT_SIDE)) {
      // Reset the pose on the Oculus side
      io.resetPose(new Pose2d());
      // Set the offset transform to the new pose
      updateTransform(oculusSidePose);
    } else {
      updateTransform(new Pose2d());
      io.resetPose(oculusSidePose);
    }
    Logger.recordOutput(
        "Oculus/Log",
        String.format("Resetting pose to WPILib: %s, Oculus: %s", pose, oculusSidePose));
    NotificationPresets.Oculus.sendOculusPoseResetNotification(pose);
  }

  /**
   * Resets the pose tracking system to a specified position. Must be called only when the robot is
   * disabled to avoid interrupting tracking during a match.
   *
   * @param pose The new reference pose
   * @param overrideEnabledCheck Overrides the enabled check to allow for resetting while enabled.
   */
  public void resetPose(Pose2d pose, boolean overrideEnabledCheck) {
    if (!overrideEnabledCheck && DriverStation.isEnabled()) {
      Logger.recordOutput(
          "Oculus/Log",
          "resetPose() called while the robot is enabled. This shouldn't happen! Ignoring.");
      return;
    }

    Logger.recordOutput(
        "Oculus/Log",
        "resetPose() called while the robot is enabled. Enabled check overridden! Make sure this is what you want to happen!");
    // Transform the pose to the Oculus coordinate system w/ offset
    Pose2d oculusSidePose = pose.plus(ROBOT_TO_OCULUS);

    if (POSE_RESET_STRATEGY.equals(PoseResetStrategy.ROBOT_SIDE)) {
      // Reset the pose on the Oculus side
      io.resetPose(new Pose2d());
      // Set the offset transform to the new pose
      updateTransform(oculusSidePose);
    } else {
      updateTransform(new Pose2d());
      io.resetPose(oculusSidePose);
    }
    Logger.recordOutput(
        "Oculus/Log",
        String.format("Resetting pose to WPILib: %s, Oculus: %s", pose, oculusSidePose));
    NotificationPresets.Oculus.sendOculusPoseResetNotification(pose);
  }

  /**
   * Resets only the heading/rotation of the Quest tracking system.
   */
  public void resetHeading() {
    if (DriverStation.isEnabled()) {
      Logger.recordOutput(
          "Oculus/Log",
          "resetHeading() called while the robot is enabled. This shouldn't happen! Ignoring.");
      return;
    }
    io.resetHeading();
    Logger.recordOutput("Oculus/Log", "Resetting heading");
  }

  /**
   * Updates the transform offset used in ROBOT_SIDE pose reset strategy. Has no effect if using a
   * different pose reset strategy.
   *
   * @param pose The new reference pose for calculating offset
   */
  public void updateTransform(Pose2d pose) {
    if (!POSE_RESET_STRATEGY.equals(PoseResetStrategy.ROBOT_SIDE)) {
      Logger.recordOutput(
          "Oculus/Log", "updateTransform() called when not using ROBOT_SIDE. Ignoring.");
      return;
    }
    // Update the offset transform to the new pose
    Logger.recordOutput("Oculus/Log", "Updating offset transform to: " + pose);
    offsetTransform = new Transform2d(pose.getTranslation(), pose.getRotation());
    NotificationPresets.Oculus.sendOculusTransformUpdateNotification(offsetTransform);
  }

  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Processes the current pose data and forwards it to the consumer if connected and properly
   * tracking. This enables integration with pose estimation systems.
   */
  private void processPose() {
    if (inputs.connected && inputs.isTracking) {
      Pose2d pose = getPose();
      double timestamp = getTimestamp();

      // Make sure we are inside the field
      if (pose.getX() < 0.0
          || pose.getX() > aprilTagFieldLayout.getFieldLength()
          || pose.getY() < 0.0
          || pose.getY() > aprilTagFieldLayout.getFieldWidth()) {
        return;
      }

      // Call the consumer with the new pose
      oculusConsumer.accept(
          CommandSwerveDrivetrain.VisionSource.OCULUS, pose, timestamp, OCULUS_STD_DEVS);
    }
  }

  /**
   * Converts the raw Oculus yaw to a Rotation2d object. Applies necessary coordinate system
   * transformations.
   *
   * @return Rotation2d representing the headset's yaw
   */
  private Rotation2d getOculusYaw() {
    return Rotation2d.fromDegrees(-inputs.eulerAngles[1]);
  }

  /**
   * Converts the raw Oculus position to a Translation2d object. Maps Unity coordinate system to FRC
   * coordinate system.
   *
   * @return Translation2d representing the headset's position
   */
  private Translation2d getOculusTranslation() {
    float[] oculusPosition = inputs.position;
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  /**
   * Combines Oculus position and orientation into a unified Pose2d.
   *
   * @return Raw Pose2d from the headset's perspective
   */
  @AutoLogOutput(key = "Oculus/RawPose")
  private Pose2d getOculusPose() {
    return new Pose2d(getOculusTranslation(), getOculusYaw());
  }

  /**
   * Functional interface for components that consume Oculus vision measurements.
   *
   * <p>Typically implemented by subsystems that handle pose estimation/odometry.
   */
  @FunctionalInterface
  public static interface OculusConsumer {
    /**
     * Accepts a vision measurement from the Oculus subsystem.
     *
     * @param source The vision source (OCULUS)
     * @param visionRobotPoseMeters Field-relative pose of the robot in meters
     * @param timestampSeconds Timestamp when the measurement was taken, in seconds
     * @param visionMeasurementStdDevs Standard deviations for the measurement (x, y, theta)
     */
    public void accept(
        CommandSwerveDrivetrain.VisionSource source,
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}