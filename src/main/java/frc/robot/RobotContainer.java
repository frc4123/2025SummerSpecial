// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

//import frc.robot.Constants.InputConstants;
import frc.robot.commands.algae_manipulator.AlgaeIntake;
import frc.robot.commands.algae_manipulator.AlgaeOutake;
import frc.robot.commands.autos.BlueLeftCoral;
import frc.robot.commands.autos.BlueRightCoral;
import frc.robot.commands.autos.MiddleCoral;
import frc.robot.commands.autos.Test;
import frc.robot.commands.coral_manipulator.CoralIntake;
import frc.robot.commands.coral_manipulator.CoralReverse;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.generated.TunerConstants;
import frc.robot.commands.swerve.DriveToPoseLeft;
import frc.robot.commands.swerve.DriveToPoseRight;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);
    // private final SwerveRequest.FieldCentricFacingAngle driveToPoseRequest = new SwerveRequest.FieldCentricFacingAngle()
    //         .withDriveRequestType(DriveRequestType.Velocity)
    //         .withSteerRequestType(SteerRequestType.Position);
        

    private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort0);
    private final CommandGenericHID m_buttonBoard = new CommandGenericHID(Constants.InputConstants.kDriverControllerPort1);

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();  
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Vision vision = new Vision(drivetrain);
    private final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    private final CoralManipulator coralManipulator = new CoralManipulator();
    private final Arm arm = new Arm();

    private final DriveToPoseRight driveToPoseRight = new DriveToPoseRight(drivetrain, vision);
    private final DriveToPoseLeft driveToPoseLeft = new DriveToPoseLeft(drivetrain, vision);
    private final AlgaeIntake algaeIntake = new AlgaeIntake(algaeManipulator);
    private final AlgaeOutake algaeOutake = new AlgaeOutake(algaeManipulator);
    private final CoralIntake coralIntake = new CoralIntake(coralManipulator);
    private final CoralReverse coralReverse = new CoralReverse(coralManipulator);

    public double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    
    public RobotContainer() {
        configureBindings();
        initializeAutoChooser();

        faceAngle.HeadingController.setP(10); 
        faceAngle.HeadingController.setI(0.0);
        faceAngle.HeadingController.setD(0.4123); 
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention
        // and Y is defined as to the left according to WPILib convention
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.x().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/10) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed/10) 
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate/10)
            )
        );
        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> faceAngle
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                .withTargetDirection(vision.getLastGamePieceAngle())
            )
        );

        joystick.rightTrigger().whileTrue(driveToPoseRight);
        joystick.leftTrigger().whileTrue(driveToPoseLeft);
        //joystick.

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.2 * MaxSpeed).withVelocityY(0))
        );
        joystick.pov(45).whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(0.3464 * MaxSpeed).withVelocityY(-0.2 * MaxSpeed))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.2 * MaxSpeed))
        );
        joystick.pov(135).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.3464 * MaxSpeed).withVelocityY(-0.2 * MaxSpeed))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.2 * MaxSpeed).withVelocityY(0))
        );
        joystick.pov(225).whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(-0.3464 * MaxSpeed).withVelocityY(0.2 * MaxSpeed))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.2 * MaxSpeed))
        );
        joystick.pov(315).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.3464 * MaxSpeed).withVelocityY(0.2 * MaxSpeed))
        );
        // 0.2, 0.3464, 60 at bottom
        // this will result in diagonal controller inputs driving the robot 60 degrees to allign with the hexagonal reef shape
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // SIGNAL LOGGER START AND STOP SHOULD BE BINDED AS WELL
        //  Commands.runOnce(SigalLogger::start)); and start should be stop for stop
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystsick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        m_buttonBoard.button(1).whileTrue(coralIntake);
        m_buttonBoard.button(2).whileTrue(coralReverse);
        m_buttonBoard.button(3).whileTrue(algaeIntake);
        m_buttonBoard.button(4).whileTrue(algaeOutake);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void initializeAutoChooser(){
        autoChooser.setDefaultOption("1 Middle Coral",new MiddleCoral().middleCoral());
        autoChooser.addOption("Test", new Test().test());
        autoChooser.addOption("Blue Coral Left 3", new BlueLeftCoral().blueLeftCoral());
        autoChooser.addOption("Blue Coral Right 3", new BlueRightCoral().blueRightCoral());
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}