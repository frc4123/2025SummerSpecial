package frc.robot.commands.swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.MathUtils;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoLineUpCoralStation extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public AutoLineUpCoralStation(CommandSwerveDrivetrain swerve, int right) {
        this.swerve = swerve;
        addRequirements(this.swerve);
    
        addCommands(
    new DeferredCommand(
        () -> (new ConditionalAllianceCommand(
            // Blue alliance command
            new CloseDriveToPose(
                swerve,
                adjustTargetYawForAlliance(
                    addRobotCentrictoFieldCentric(
                        MathUtils.findClosestTarget(
                            this.swerve.getState().Pose,
                            Constants.AutoDriveConstants.BLUE_CORALSTATION_POSES,
                            false // false for blue alliance
                        ),
                        right, // int parameter
                        false // false for blue alliance
                    ),
                    false // false for blue alliance
                )
            ),
            // Red alliance command
            new CloseDriveToPose(
                swerve,
                adjustTargetYawForAlliance(
                    addRobotCentrictoFieldCentric(
                        MathUtils.findClosestTarget(
                            this.swerve.getState().Pose,
                            Constants.AutoDriveConstants.RED_CORALSTATION_POSES,
                            true // true for red alliance
                        ),
                        right, // int parameter
                        true // true for red alliance
                    ),
                    true // true for red alliance
                )
            )
        )),
        getRequirements()
    )
);
    }
    
    private Pose2d adjustTargetYawForAlliance(Pose2d targetPose, boolean isRedAlliance) {
        if (isRedAlliance) {
            // Flip the yaw by 180 degrees for the red alliance
            return new Pose2d(
                targetPose.getX(),
                targetPose.getY(),
                targetPose.getRotation().plus(Rotation2d.fromDegrees(180))
            );
        }
        return targetPose; // No adjustment for blue alliance
    }

    public Pose2d addRobotCentrictoFieldCentric(Pose2d robotPose, int right, boolean isRedAlliance) {
        double xOffset = Constants.AutoDriveConstants.CORALSTATIONADDITIONS[right][0];
        double yOffset = Constants.AutoDriveConstants.CORALSTATIONADDITIONS[right][1];
    
        // Flip the X and Y offsets for the red alliance
        if (isRedAlliance) {
            xOffset *= -1;
            yOffset *= -1; // Flip the Y offset as well
        }
    
        double newX = robotPose.getX() + (xOffset * Math.cos(robotPose.getRotation().getRadians()) - yOffset * Math.sin(robotPose.getRotation().getRadians()));
        double newY = robotPose.getY() + (xOffset * Math.sin(robotPose.getRotation().getRadians()) + yOffset * Math.cos(robotPose.getRotation().getRadians()));
        Pose2d calculated = new Pose2d(newX, newY, robotPose.getRotation());
        return calculated;
    }
}