package frc.robot.commands.autos;


import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

public class MiddleCoral extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    public MiddleCoral(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Set up the PPHolonomicDriveController
        PIDConstants translationConstants = new PIDConstants(3.0, 0.0, 0.0); // Translation PID
        PIDConstants rotationConstants = new PIDConstants(1.7, 0.0, 0.0); // Rotation PID
        PPHolonomicDriveController driveController = new PPHolonomicDriveController(translationConstants, rotationConstants);

        // Define the robot's mass and moment of inertia
        double massKG = 20.0; // Replace with your robot's mass in kilograms
        double MOI = 1.0; // Replace with your robot's moment of inertia in kg·m²

        //Define the module configuration
        ModuleConfig moduleConfig = new ModuleConfig(
            0.1, // Wheel radius in meters (replace with your wheel radius)
            4,  // Max drive speed in meters per second (replace with your max speed)
            0.8,  // Coefficient of friction (replace with your wheel's COF)
            DCMotor.getKrakenX60(2), // Replace with your drive motor type
            60.0, // Drive current limit in amps (replace with your motor's current limit)
            2     // Number of motors per module (replace with your configuration)
        );

        // TO CONFIGURE Define the module offsets (positions relative to the robot's center)     
        Translation2d[] moduleOffsets = {
            new Translation2d(0.314325, 0.314325),  // Front Left (replace with your module's position)
            new Translation2d(0.314325, -0.314325), // Front Right (replace with your module's position)
            new Translation2d(-0.314325, 0.314325), // Back Left (replace with your module's position)
            new Translation2d(-0.314325, -0.314325) // Back Right (replace with your module's position)
        };

        // Create the RobotConfig object
        RobotConfig robotConfig = new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);

        // Configure the AutoBuilder
        AutoBuilder.configure(
            drivetrain::getPose2d, // Replace with correct getter for Pose2d
            drivetrain::resetOdometer, // Ensure resetOdometer accepts Pose2d
            drivetrain::getRobotRelativeSpeeds, // Ensure this returns correct type
            drivetrain::driveRobotRelative, // Ensure method signature matches
            driveController, // okay
            robotConfig, // okay
            () -> false, // okay
            drivetrain
            );
    }

    public Command middleCoral() {
        return AutoBuilder.buildAuto("middleCoral");
    }
}