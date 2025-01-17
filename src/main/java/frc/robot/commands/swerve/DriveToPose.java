package frc.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    public DriveToPose(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // PID controllers for X, Y, and theta (rotation)
        xController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(3.0, 3.0)); // Adjust gains
        yController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(3.0, 3.0)); // Adjust gains
        thetaController = new ProfiledPIDController(1.7, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI)); // Adjust gains
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // For rotation

        faceAngle.HeadingController.setP(3);
        faceAngle.HeadingController.setI(0.0);
        faceAngle.HeadingController.setD(0.01); 
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        // Reset the controllers to the current pose
        Pose2d currentPose = drivetrain.getState().Pose; // Use odometry or vision-based pose
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
    }

    @Override
    public void execute() {
        // Get the current pose (use vision if available, otherwise use odometry)
        if (vision.hasTarget()){
            Pose2d currentPose = vision.get2dPose() != null ? vision.get2dPose() : drivetrain.getState().Pose;
            double xTarget = vision.getTargetPose2d() != null ? vision.getTargetPose2d().getX() : 0;
            double yTarget = vision.getTargetPose2d() != null ? vision.getTargetPose2d().getY() : 0;

            // Calculate the target speeds using the controllers
            double xSpeed = xController.calculate(currentPose.getX(), xTarget);
            double ySpeed = yController.calculate(currentPose.getY(), yTarget);

            drivetrain.applyRequest(() -> faceAngle
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed));
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> faceAngle
            .withVelocityX(0)
            .withVelocityY(0));
    }
}