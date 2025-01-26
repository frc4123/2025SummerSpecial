package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToPose2 extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private Pose2d targetPose;
    private Command pathCommand;

    // Create an instance of SwerveRequest.Idle to stop the drivetrain
    private final SwerveRequest.Idle stopRequest = new SwerveRequest.Idle();

    public DriveToPose2(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        targetPose = vision.getTargetPose2d();

        PathConstraints constraints = new PathConstraints(
            4.0, // maxVelocityMPS
            4.0, // maxAccelerationMPSSq
            Math.PI, // maxAngularVelocityRadPerSec
            Math.PI // maxAngularAccelerationRadPerSecSq
        );

        pathCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            targetPose.getRotation().getRadians()
        );

        pathCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand != null && pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }

        // Stop the swerve drivetrain by applying the Idle request
        drivetrain.setControl(stopRequest);
    }
}