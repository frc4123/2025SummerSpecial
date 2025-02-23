package frc.robot.commands.swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utils.ConditionalAllianceCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoLineUpProcessor extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public AutoLineUpProcessor(CommandSwerveDrivetrain swerve) {

        this.swerve = swerve;

        addRequirements(this.swerve);

        // Add conditional logic to choose commands
        addCommands(
            new DeferredCommand(
                () -> new ConditionalAllianceCommand(
                    new CloseDriveToPose(swerve, new Pose2d(6.600, 0.578, new Rotation2d(-90))), // Blue alliance command
                    new CloseDriveToPose(swerve, new Pose2d(11.412, 7.486, new Rotation2d(90))) // Red alliance command
                ),
                getRequirements()
            )
        );
    }
}