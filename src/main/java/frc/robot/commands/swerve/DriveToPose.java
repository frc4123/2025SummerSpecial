package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    private Pose2d targetPose;

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public DriveToPose(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Ensure the drivetrain and vision subsystems are required
        addRequirements(drivetrain, vision);
        
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // The AutoBuilder handles the execution of the pathfinding and driving logic
        // No need to manually calculate speeds or apply requests

        // Get the target pose from vision
        if (vision.hasTarget()) {
            targetPose = vision.getTargetPose2d();
        } else {
            // If no vision target, use the current pose as a fallback
            targetPose = drivetrain.getState().Pose;
        }

        // Use AutoBuilder to drive to the target pose
        if (targetPose != null) {
            // Define path constraints (max velocity, max acceleration, max angular velocity)
            PathConstraints constraints = new PathConstraints(
                3.0, // Max velocity (m/s)
                3.0, // Max acceleration (m/s^2)
                Math.PI, // Max angular velocity (rad/s)
                Math.PI // Max angular acceleration (rad/s^2)
            );

            // Use AutoBuilder to create a path to the target pose
            AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0

            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivetrain.applyRequest(() -> brake);
    }

    @Override
    public boolean isFinished() {
        // Check if the robot has reached the target pose
        Pose2d currentPose = drivetrain.getState().Pose;
        return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.05 // Position tolerance (m)
            && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians()) < 0.05; // Rotation tolerance (rad)
    }
}