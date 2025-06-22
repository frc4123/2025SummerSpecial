package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Quest.utils.RollingAveragePose2d;

/**
 * Interface with the QuestNav on VR headset for pose estimation. See
 * https://www.chiefdelphi.com/t/questnav-the-best-robot-pose-tracking-system-in-frc/
 */
public class OculusQ {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Pose of the Quest when the pose was reset
  private Pose2d resetPoseOculus = new Pose2d();

  // Pose of the robot when the pose was reset
  private Pose2d resetPoseRobot = new Pose2d();

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }

  private final Transform2d robotToQuest =
      new Transform2d(inchesToMeters(0.5), inchesToMeters(9.207), Rotation2d.fromDegrees(90));

  private final RollingAveragePose2d rollingAvg;

  /* Constructor */
  public OculusQ(int windowSize) {
    // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }

    rollingAvg = new RollingAveragePose2d(windowSize);
  }

  public OculusQ() {
    this(2);
  }

  public void updateAverageRobotPose() {
    rollingAvg.addPose(getRobotPose());
  }

  public Pose2d getAverageRobotPose() {
    return rollingAvg.getAveragePose();
  }

  /**
   * Gets the pose of the robot on the field
   *
   * @return pose of the robot
   */
  public Pose2d getRobotPose() {
    // The robot is the Quest's pose transformed back by the quest->robot offset
    return getQuestPose().transformBy(robotToQuest.inverse());
  }

  /**
   * Gets the pose of the Quest on the field
   *
   * @return pose of the Quest
   */
  public Pose2d getQuestPose() {
    var rawPose = getUncorrectedOculusPose();
    var poseRelativeToReset = rawPose.minus(resetPoseOculus);
    // Transform from "reset quest pose" to "current quest pose"

    return resetPoseRobot // the robot's field pose at reset
        .transformBy(robotToQuest) // offset to get the Quest's field pose at reset
        .transformBy(poseRelativeToReset);
  }

  /*
   * Gets the battery percent of the Quest.
   *
   * @return battery percent of the Quest
   */
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  public long getTime(){
    return RobotController.getFPGATime();
  }

  /**
   * Returns if the Quest is connected
   *
   * @return true if the Quest is connected
   */
  public boolean isConnected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000.0) < 30.0;
  }

  /**
   * Gets the raw Rotation3d of the Quest
   *
   * @return Rotation3d of the Quest, not adjusted for the reset pose
   */
  public Rotation3d getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Rotation3d(new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]));
  }

  /**
   * Gets the Quests's timestamp
   *
   * @return the Quest timestamp
   */
  public double getTimestamp() {
    return questTimestamp.get();
  }

  /**
   * Set the robot's pose on the field. This is useful to seed the robot to a known position. This
   * is usually called at the start of the autonomous period.
   *
   * @param newPose new robot pose
   */
  public void resetPose(Pose2d newPose) {
    rollingAvg.reset();
    resetPoseOculus = getUncorrectedOculusPose();
    resetPoseRobot = newPose;
  }

  /**
   * Clean up questnav subroutine messages after processing on the headset. Call this each iteration
   * to reset the command after it has been processed.
   */
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  /**
   * Gets the raw pose of the oculus, relative to the position where it started
   *
   * @return pose of the oculus
   */
  private Pose2d getUncorrectedOculusPose() {
    var eulerAngles = questEulerAngles.get();
    var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

    var questnavPosition = questPosition.get();
    var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
    return new Pose2d(translation, rotation);
  }
}