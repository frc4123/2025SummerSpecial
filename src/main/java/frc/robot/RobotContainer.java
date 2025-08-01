// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.algae_manipulator.AlgaeIntake;
import frc.robot.commands.algae_manipulator.AlgaeIntakeStop;
import frc.robot.commands.algae_manipulator.AlgaeOutake;
import frc.robot.commands.arm.ArmBarge;
import frc.robot.commands.arm.ArmOut;
import frc.robot.commands.arm.ArmProcessor;
import frc.robot.commands.arm.ArmStow;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.arm.ArmReef;
import frc.robot.commands.autos.CoralLeft3;
import frc.robot.commands.autos.CoralRight3;
import frc.robot.commands.autos.LeftLeave;
import frc.robot.commands.autos.MiddleCoral;
import frc.robot.commands.autos.mtest;
import frc.robot.commands.coral_manipulator.CheckIntake;
import frc.robot.commands.coral_manipulator.CoralIntake;
import frc.robot.commands.coral_manipulator.CoralIntakeStop;
import frc.robot.commands.coral_manipulator.CoralReverse;
import frc.robot.commands.coral_manipulator.CoralYolo;
import frc.robot.commands.elevator.ElevatorL2;
import frc.robot.commands.elevator.ElevatorL2Algae;
import frc.robot.commands.elevator.Elevator3CoralAuto;
import frc.robot.commands.elevator.ElevatorAlgaeGround;
import frc.robot.commands.elevator.ElevatorBarge;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorL3;
import frc.robot.commands.elevator.ElevatorL4;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.generated.TunerConstants;
import frc.robot.commands.swerve.AutoLineUpAlgae;
import frc.robot.commands.swerve.AutoLineUpCoralStation;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Oculus;
import frc.robot.subsystems.Vision;

public class RobotContainer {
   // private final double MaxSpeedinit = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.RobotCentric robotStrafe = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort0);
    private final CommandGenericHID m_buttonBoard = new CommandGenericHID(Constants.InputConstants.kDriverControllerPort1);

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();  
    private final Oculus oculus = new Oculus();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Vision vision = new Vision(drivetrain, oculus);
    private final AlgaeManipulator algaeManipulator = new AlgaeManipulator();
    private final CoralManipulator coralManipulator = new CoralManipulator();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();

    private final AlgaeIntake algaeIntake = new AlgaeIntake(algaeManipulator);
    private final AlgaeOutake algaeOutake = new AlgaeOutake(algaeManipulator);
    private final AlgaeIntakeStop algaeIntakeStop = new AlgaeIntakeStop(algaeManipulator);
    private final CoralIntake coralIntake = new CoralIntake(coralManipulator, elevator);
    private final CoralReverse coralReverse = new CoralReverse(coralManipulator);
    private final CoralIntakeStop coralIntakeStop = new CoralIntakeStop(coralManipulator);
    private final CoralYolo coralYolo = new CoralYolo(coralManipulator, elevator);
    private final CheckIntake checkIntake = new CheckIntake(coralManipulator);
    private final ArmStow armStow = new ArmStow(arm);
    private final ArmOut armOut = new ArmOut(arm);
    private final ArmReef armReef = new ArmReef(arm);
    private final ArmBarge armBarge = new ArmBarge(arm);
    private final ArmProcessor armProcessor = new ArmProcessor(arm);
    private final ArmUp armUp = new ArmUp(arm);
    private final ElevatorDown elevatorDown = new ElevatorDown(elevator);
    private final ElevatorAlgaeGround elevatorAlgaeGround = new ElevatorAlgaeGround(elevator);
    private final ElevatorBarge elevatorBarge = new ElevatorBarge(elevator);
    private final ElevatorL2Algae elevatorL2Algae = new ElevatorL2Algae(elevator);
    private final ElevatorL2 elevatorL2 = new ElevatorL2(elevator);
    private final ElevatorL3 elevatorL3 = new ElevatorL3(elevator);
    private final ElevatorL4 elevatorL4 = new ElevatorL4(elevator, arm);
    private final Elevator3CoralAuto elevatorL4auto = new Elevator3CoralAuto(elevator, arm);

    private final Command leftCoralAutoDrive = new AutoLineUpReef(drivetrain, 0);
    private final Command rightCoralAutoDrive = new AutoLineUpReef(drivetrain, 1);
    private final Command algaeAutoDrive = new AutoLineUpAlgae(drivetrain);
    //private final Command leftCoralStationAutoDrive = new AutoLineUpCoralStation(drivetrain, 0);
    private final Command rightCoralStationAutoDrive = new AutoLineUpCoralStation(drivetrain, 1);

    public double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    
    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("CoralYolo", coralYolo);
        NamedCommands.registerCommand("Intake", coralIntake);
        NamedCommands.registerCommand("CheckIntake", checkIntake);
        NamedCommands.registerCommand("ElevatorDown", elevatorDown);
        NamedCommands.registerCommand("ScoreL4", elevatorL4auto);

        initializeAutoChooser();
        
        faceAngle.HeadingController.setP(5);  // 10
        faceAngle.HeadingController.setI(0.0);
        faceAngle.HeadingController.setD(0);  // 0.4123
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void configureBindings() {

        if (RobotBase.isSimulation()) {
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.6 * elevator.getElevatorSwerveReduction()) 
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * elevator.getElevatorSwerveReduction()) 
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
                )
            );
        } else {
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.6 * elevator.getElevatorSwerveReduction()) 
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * elevator.getElevatorSwerveReduction()) 
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
                )
            );
        }


        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.x().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/4) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed/4) 
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate/4)
            )
        );
        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> faceAngle
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                .withTargetDirection(vision.getLastGamePieceAngle())
            )
        );

        joystick.rightTrigger().whileTrue(rightCoralAutoDrive);
        joystick.leftTrigger().whileTrue(leftCoralAutoDrive);
        joystick.y().whileTrue(algaeAutoDrive);
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.leftBumper().whileTrue(leftCoralStationAutoDrive);
        joystick.rightBumper().whileTrue(rightCoralStationAutoDrive);

        /*joystick.x().and(joystick.povLeft()).whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(MaxSpeed)
            .withVelocityX(0)));

        joystick.x().and(joystick.povRight()).whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(-MaxSpeed)
            .withVelocityX(0)));
        
        joystick.x().and(joystick.povUp()).whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(MaxSpeed)
            .withVelocityY(0)));

        joystick.x().and(joystick.povDown()).whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(-MaxSpeed)
            .withVelocityY(0)));
        */
        //fast mode up there
        //slow mode below

        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(MaxSpeed * 0.25)
            .withVelocityX(0)));

        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(-MaxSpeed * 0.25)
            .withVelocityX(0)));
        
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(MaxSpeed * 0.25)
            .withVelocityY(0)));

                // Replace your existing povDown() binding with this:
        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> {
            // Get current robot rotation
            double robotAngle = vision.getLastGamePieceAngle().getRadians();
    
            // Convert robot-centric "backward" movement to field-centric coordinates
            double fieldX = -MaxSpeed * 0.25 * Math.cos(robotAngle);
            double fieldY = -MaxSpeed * 0.25 * Math.sin(robotAngle);
    
            return faceAngle
                .withVelocityX(fieldX)
                .withVelocityY(fieldY)
                .withTargetDirection(vision.getLastGamePieceAngle());
        }));
    
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); 

        
        
        // joystick.leftTrigger().whileTrue(Commands.runOnce(SignalLogger::start));
        // joystick.rightTrigger().whileTrue(Commands.runOnce(SignalLogger::stop));
        // joystick.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        m_buttonBoard.button(1).onTrue(coralReverse);
        m_buttonBoard.button(1).onFalse(coralIntakeStop);

        m_buttonBoard.button(2).onTrue(algaeOutake);
        m_buttonBoard.button(2).onFalse(algaeIntakeStop);

        m_buttonBoard.button(3).onTrue(algaeIntake);
        m_buttonBoard.button(3).onTrue(elevatorAlgaeGround);
        m_buttonBoard.button(3).onTrue(armOut);
        m_buttonBoard.button(3).onFalse(elevatorDown);
        m_buttonBoard.button(3).onFalse(armStow);
        m_buttonBoard.button(3).onFalse(algaeIntakeStop);
        
        m_buttonBoard.button(4).onTrue(elevatorAlgaeGround);
        m_buttonBoard.button(4).onTrue(armProcessor);
        m_buttonBoard.button(4).onFalse(elevatorDown);
        m_buttonBoard.button(4).onFalse(armStow);
        m_buttonBoard.button(4).onFalse(algaeIntakeStop);

        m_buttonBoard.button(5).onTrue(algaeIntake);
        m_buttonBoard.button(5).whileTrue(drivetrain.applyRequest(() 
            -> robotStrafe
                .withVelocityX(0.04 * MaxSpeed)
                .withVelocityY(0))
                .withTimeout(3));
        m_buttonBoard.button(5).onTrue(elevatorL2Algae);
        m_buttonBoard.button(5).onTrue(armReef);
        m_buttonBoard.button(5).onFalse(elevatorDown);
        m_buttonBoard.button(5).onFalse(armStow);
        m_buttonBoard.button(5).onFalse(algaeIntakeStop);

        m_buttonBoard.button(6).onTrue(algaeIntake);
        m_buttonBoard.button(6).whileTrue(drivetrain.applyRequest(() 
            -> robotStrafe
                .withVelocityX(0.04 * MaxSpeed)
                .withVelocityY(0))
                .withTimeout(3));
        m_buttonBoard.button(6).onTrue(elevatorL3);
        m_buttonBoard.button(6).onTrue(armReef);
        m_buttonBoard.button(6).onFalse(elevatorDown);
        m_buttonBoard.button(6).onFalse(armStow);
        m_buttonBoard.button(6).onFalse(algaeIntakeStop);

        m_buttonBoard.button(7).onTrue(elevatorBarge);
        m_buttonBoard.button(7).onTrue(armUp);
        m_buttonBoard.button(7).onTrue(armBarge);
        m_buttonBoard.button(7).onFalse(elevatorDown);
        m_buttonBoard.button(7).onFalse(armStow);
        m_buttonBoard.button(7).onFalse(algaeIntakeStop);

        m_buttonBoard.button(8).onTrue(coralIntake);
        m_buttonBoard.button(8).onFalse(coralIntakeStop);

        m_buttonBoard.povLeft().onTrue(elevatorL4); // green (4)
        m_buttonBoard.povLeft().onFalse(armStow);
        m_buttonBoard.povDown().onTrue(elevatorL2); // blue (2) // find these and order them with L1 being bottom button
        m_buttonBoard.povUp().onTrue(armBarge); 
        m_buttonBoard.povUp().onTrue(new WaitCommand(0.8).andThen(algaeOutake)); 
        m_buttonBoard.povUp().onFalse(armStow); 
        m_buttonBoard.povUp().onFalse(algaeIntakeStop); 
        m_buttonBoard.povRight().onTrue(elevatorL3); // yellow (3)

        m_buttonBoard.povCenter().onTrue(elevatorDown);

        // end of finalized commands
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void initializeAutoChooser() {
        autoChooser.setDefaultOption("Right 3 Coral", new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new CoralRight3().coralRight3())
        ));

        autoChooser.addOption("Middle Coral (haven't tested)" , new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new MiddleCoral().middleCoral())));

        autoChooser.addOption("5m test", new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new mtest().metertest())));

        autoChooser.addOption("Left 3 Coral", new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new CoralLeft3().coralLeft3())
        ));
        
        autoChooser.addOption("Left Leave", new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new LeftLeave().leftLeave())
        ));

        // autoChooser.addOption("5 meter test", new ParallelCommandGroup(
        // new WaitCommand(0.01),
        //   new SequentialCommandGroup(new mtest().metertest())
        // ));

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}