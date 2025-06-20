package frc.robot.commands.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.experimental.UtilityClass;

@UtilityClass
public class DashboardNotifs {
    public final class Oculus {
    /** Tracks whether a battery low notification has already been sent */
    private static boolean oculusBatteryLowNotificationSent = false;

    /** Tracks whether a battery critical notification has already been sent */
    private static boolean oculusBatteryCriticalNotificationSent = false;

    /** Tracks whether a disconnection notification has already been sent */
    private static boolean oculusDisconnectedNotificationSent = false;

    /** Tracks whether a tracking lost notification has already been sent */
    private static boolean oculusTrackingLostNotificationSent = false;

    /** Sends a notification when the Oculus pose reset fails. */
    public static void sendOculusPoseResetFailedNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Pose Reset Failed",
              "Not using Quest",
              3000));
    }

    /**
     * Sends a notification when the Oculus pose is reset.
     *
     * @param newPose The new pose after reset
     */
    public static void sendOculusPoseResetNotification(Pose2d newPose) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Pose Reset",
              "New Position: "
                  + newPose.getTranslation().toString()
                  + newPose.getRotation().getDegrees()
                  + "deg",
              3000));
    }

    /** Sends a notification when the Oculus heading reset fails. */
    public static void sendOculusHeadingResetFailedNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Heading Reset Failed",
              "Not using Quest",
              3000));
    }

    /** Sends a notification when the Oculus heading is reset. */
    public static void sendOculusHeadingResetNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Heading Reset",
              "Reset complete",
              3000));
    }

    /**
     * Sends a notification when the Oculus transform is updated.
     *
     * @param newTransform The new transform applied to the Oculus
     */
    public static void sendOculusTransformUpdateNotification(Transform2d newTransform) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Transform Update",
              "New Position: "
                  + newTransform.getTranslation().toString()
                  + newTransform.getRotation().getDegrees()
                  + "deg",
              3000));
    }

    /**
     * Sends a notification when the Oculus reconnects. Only sends if a disconnection was previously
     * reported.
     */
    public static void sendOculusReconnectedNotification() {
      // Don't send the notification if the Oculus has never been disconnected
      if (!oculusDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Reconnected",
              "The Oculus has been reconnected.",
              3000));
      oculusDisconnectedNotificationSent = false;
    }

    /**
     * Sends a notification when the Oculus disconnects. Only sends the notification once until the
     * device reconnects.
     */
    public static void sendOculusDisconnectedNotification() {
      // Only send the notification once
      if (oculusDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Disconnected",
              "The Oculus has been disconnected.",
              3000));
      oculusDisconnectedNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus battery is low. Only sends the notification once per
     * battery discharge cycle.
     */
    public static void sendOculusBatteryLowNotification() {
      // Only send the notification once
      if (oculusBatteryLowNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Battery Low",
              "The Oculus battery is low.",
              3000));
      oculusBatteryLowNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus battery is critically low. Only sends the notification
     * once per battery discharge cycle.
     */
    public static void sendOculusBatteryCriticalNotification() {
      // Only send the notification once
      if (oculusBatteryCriticalNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Battery Critical",
              "The Oculus battery is critical.",
              3000));
      oculusBatteryCriticalNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus tracking is lost. Only sends the notification once until
     * tracking is regained.
     *
     * @param totalTrackingLostEvents The times the quest has lost tracking total since app boot.
     */
    public static void sendOculusTrackingLostNotification(int totalTrackingLostEvents) {
      // Only send the notification once
      if (oculusTrackingLostNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Tracking Lost",
              String.format(
                  "Oculus Tracking Lost. (%d time(s) this boot)", totalTrackingLostEvents),
              3000));
      oculusTrackingLostNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus tracking is regained. Only sends the notification once
     * until tracking is lost again.
     */
    public static void sendOculusTrackingRegainedNotification() {
      // Don't send the notification if the tracking has never been lost
      if (!oculusTrackingLostNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Tracking Regained",
              "Oculus Tracking Regained. Switching back to Oculus tracking.",
              3000));
      oculusTrackingLostNotificationSent = false;
    }
  }
}
