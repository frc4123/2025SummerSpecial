package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import frc.robot.Constants.OculusQuest;
import frc.robot.commands.utils.DashboardNotifs;

import gg.questnav.questnav.QuestNav;

public class Oculus{

  // Pose of the Quest when the pose was reset
  private Pose2d resetPoseOculus = new Pose2d();

  private QuestNav quest = new QuestNav();

  private final Transform2d robotToQuest = new Transform2d(inchesToMeters(0.5), inchesToMeters(9.207), Rotation2d.fromDegrees(90));

  //private final RollingAveragePose2d rollingAvg;

  /* Constructor */
  public Oculus() {
  }

  // public void updateAverageRobotPose() {
  //   rollingAvg.addPose(getRobotPose());
  // }

  // public Pose2d getAverageRobotPose() {
  //   return rollingAvg.getAveragePose();
  // }

  /**
   * Gets the pose of the robot on the field
   *
   * @return pose of the robot
   */
  public Pose2d getRobotPose() {
    // The robot is the Quest's pose transformed back by the quest->robot offset
    return quest.getPose().transformBy(robotToQuest.inverse());
  }

  /*
   * Gets the battery percent of the Quest.
   *
   * @return battery percent of the Quest
   */
  public double getBatteryPercent() {
    return quest.getBatteryPercent();
  }

  public void resetPoseOculus(Pose2d lastPose){
    resetPoseOculus = lastPose.transformBy(OculusQuest.ROBOT_TO_OCULUS);

    quest.setPose(resetPoseOculus);
  }

  // public long getTime(){
  //   return RobotController.getFPGATime();
  // }

  public double getTime(){
    return quest.getDataTimestamp();
  }

  /**
   * Returns if the Quest is connected
   *
   * @return true if the Quest is connected
   */
  public boolean isConnected() {
    return quest.getAppTimestamp() != -1;
  }

  public boolean isTracking(){
    return quest.isTracking();
  }

  public Pose2d getRawPose(){
    return quest.getPose();
  }
  /**
   * Gets the raw Rotation2d of the Quest
   *
   * @return Rotation2d of the Quest, not adjusted for the reset pose
   */
  public Rotation2d getRotation() {
    return quest.getPose().getRotation();
  }

  public Pose2d getRobotPose2d(){
    Pose2d questPose = quest.getPose();
    return questPose.transformBy(OculusQuest.ROBOT_TO_OCULUS.inverse());
  }

  public void sendHeartbeat(){
    quest.commandPeriodic();
  }

  public void processOculusNotifications(){
    // Notify if we are disconnected
    if (!quest.isConnected()) {
        DashboardNotifs.Oculus.sendOculusDisconnectedNotification();
    } else {
        DashboardNotifs.Oculus.sendOculusReconnectedNotification();
    }

    // Notify for battery levels
    if (quest.getBatteryPercent() < OculusQuest.BATTERY_CRITICAL_PERCENT) {
        DashboardNotifs.Oculus.sendOculusBatteryCriticalNotification();
    } else if (quest.getBatteryPercent() < OculusQuest.BATTERY_CRITICAL_PERCENT) {
        DashboardNotifs.Oculus.sendOculusBatteryLowNotification();
    }
  }
}