// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

//import frc.robot.Constants.InputConstants;
import frc.robot.commands.algae_manipulator.AlgaeIntake;
import frc.robot.commands.algae_manipulator.AlgaeIntakeStop;
import frc.robot.commands.algae_manipulator.AlgaeOutake;
import frc.robot.commands.arm.ArmBarge;
import frc.robot.commands.arm.ArmOut;
import frc.robot.commands.arm.ArmProcessor;
import frc.robot.commands.arm.ArmStow;
import frc.robot.commands.arm.ArmUp;
import frc.robot.commands.arm.ArmReef;
// import frc.robot.commands.autos.BlueLeftCoral2;
import frc.robot.commands.autos.BlueRight2Coralv2;
// import frc.robot.commands.autos.BlueRightCoral2;
import frc.robot.commands.autos.LeftLeave;
import frc.robot.commands.autos.MiddleCoral;
import frc.robot.commands.coral_manipulator.CoralIntake;
import frc.robot.commands.coral_manipulator.CoralIntakeStop;
import frc.robot.commands.coral_manipulator.CoralReverse;
import frc.robot.commands.elevator.ElevatorL2;
import frc.robot.commands.elevator.ElevatorL2Algae;
import frc.robot.commands.elevator.Elevator2CoralAuto;
import frc.robot.commands.elevator.ElevatorAlgaeGround;
import frc.robot.commands.elevator.ElevatorAutoVision;
import frc.robot.commands.elevator.ElevatorBarge;
import frc.robot.commands.elevator.ElevatorDown;
import frc.robot.commands.elevator.ElevatorL1;
import frc.robot.commands.elevator.ElevatorL3;
import frc.robot.commands.elevator.ElevatorL4;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.generated.TunerConstants;
import frc.robot.commands.swerve.AutoLineUpAlgae;
import frc.robot.commands.swerve.AutoLineUpReef;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.RobotCentric robotStrafe = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
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
    private final Elevator elevator = new Elevator();

    // private final DriveToPoseRight driveToPoseRight = new DriveToPoseRight(drivetrain, vision);
    // private final DriveToPoseLeft driveToPoseLeft = new DriveToPoseLeft(drivetrain, vision);
    private final AlgaeIntake algaeIntake = new AlgaeIntake(algaeManipulator);
    private final AlgaeOutake algaeOutake = new AlgaeOutake(algaeManipulator);
    private final AlgaeIntakeStop algaeIntakeStop = new AlgaeIntakeStop(algaeManipulator);
    private final CoralIntake coralIntake = new CoralIntake(coralManipulator, elevator);
    private final CoralReverse coralReverse = new CoralReverse(coralManipulator);
    private final CoralIntakeStop coralIntakeStop = new CoralIntakeStop(coralManipulator);
    private final ArmStow armStow = new ArmStow(arm);
    private final ArmOut armOut = new ArmOut(arm);
    private final ArmReef armReef = new ArmReef(arm);
    private final ArmBarge armBarge = new ArmBarge(arm);
    private final ArmProcessor armProcessor = new ArmProcessor(arm);
    private final ArmUp armUp = new ArmUp(arm);
    private final ElevatorDown elevatorDown = new ElevatorDown(elevator);
    private final ElevatorAlgaeGround elevatorAlgaeGround = new ElevatorAlgaeGround(elevator);
    private final ElevatorBarge elevatorBarge = new ElevatorBarge(elevator);
    private final ElevatorL1 elevatorL1 = new ElevatorL1(elevator);
    private final ElevatorL2Algae elevatorL2Algae = new ElevatorL2Algae(elevator);
    private final ElevatorL2 elevatorL2 = new ElevatorL2(elevator);
    private final ElevatorL3 elevatorL3 = new ElevatorL3(elevator);
    private final ElevatorL4 elevatorL4 = new ElevatorL4(elevator, arm);

    private final Command leftCoralAutoDrive = new AutoLineUpReef(drivetrain, 0);
    private final Command rightCoralAutoDrive = new AutoLineUpReef(drivetrain, 1);
    private final Command algaeAutoDrive = new AutoLineUpAlgae(drivetrain);

    public double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
    
    public RobotContainer() {
        configureBindings();
        initializeAutoChooser();

        faceAngle.HeadingController.setP(5);  // 10
        faceAngle.HeadingController.setI(0.0);
        faceAngle.HeadingController.setD(0);  // 0.4123
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        NamedCommands.registerCommand("ElevatorL4", elevatorL4);
        NamedCommands.registerCommand("CoralIntake", coralIntake);
        NamedCommands.registerCommand("CoralIntakeStop", coralIntakeStop);
        NamedCommands.registerCommand("ElevatorDown", elevatorDown);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention
        // and Y is defined as to the left according to WPILib convention
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * elevator.getElevatorSwerveReduction()) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * elevator.getElevatorSwerveReduction()) 
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

        joystick.rightTrigger().whileTrue(rightCoralAutoDrive);
        joystick.leftTrigger().whileTrue(leftCoralAutoDrive);
        joystick.y().whileTrue(algaeAutoDrive);

        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(0.1 * MaxSpeed)
            .withVelocityX(0)));

        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(-0.1 * MaxSpeed)
            .withVelocityX(0)));
        
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(0.1 * MaxSpeed)
            .withVelocityY(0)));

        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(-0.1 * MaxSpeed)
            .withVelocityY(0)));
    
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Final Button Mappings below

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
        m_buttonBoard.button(5).onTrue(elevatorL2Algae);
        m_buttonBoard.button(5).onTrue(armReef);
        m_buttonBoard.button(5).onFalse(elevatorDown);
        m_buttonBoard.button(5).onFalse(armStow);
        m_buttonBoard.button(5).onFalse(algaeIntakeStop);

        m_buttonBoard.button(6).onTrue(algaeIntake);
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
        m_buttonBoard.povUp().onTrue(elevatorL1); // red (1)
        m_buttonBoard.povRight().onTrue(elevatorL3); // yellow (3)

        m_buttonBoard.povCenter().onTrue(elevatorDown);

        // end of finalized commands
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void initializeAutoChooser(){

        autoChooser.setDefaultOption("1 Middle Coral" , new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new MiddleCoral().middleCoral()),
          new SequentialCommandGroup(
                new CoralIntake(coralManipulator, elevator).withTimeout(5)
                .andThen(new ElevatorL4(elevator, arm).withTimeout(2))
                .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.5))
                .andThen(new ElevatorDown(elevator).withTimeout(2))
        )));

        // autoChooser.addOption("Right 2 Coral", new ParallelCommandGroup(
        // new WaitCommand(0.01),
        //   new SequentialCommandGroup(new BlueRightCoral2().blueRightCoral()),
        //   new SequentialCommandGroup(
        //         new CoralIntake(coralManipulator, elevator).withTimeout(2.54)
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.6))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.6))
        //         // intake and score first coral
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(4.13 + 0.1))
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.6))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.6))
        //         //intake and score second coral
        // )));

        // autoChooser.addOption("Right 2 Coral", new ParallelCommandGroup(
        // new WaitCommand(0.01),
        //   new SequentialCommandGroup(new BlueRightCoral2().blueRightCoral()),
        //   new SequentialCommandGroup(
        //         new CoralIntake(coralManipulator, elevator).withTimeout(2.12) // 2.54 -
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.7))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.7))
        //         // intake and score first coral
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(4.13 + 0.1))
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.7))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.7))
        //         //intake and score second coral
        // )));

        // autoChooser.addOption("(don't use) Right 2 Coral Drive To Pose", new ParallelCommandGroup(
        // new WaitCommand(0.01),
        //   new SequentialCommandGroup(new BlueRightCoral2().blueRightCoral()
        //   .andThen(new WaitCommand(9.84).andThen(new AutoLineUpReef(drivetrain,1)))),
        //   new SequentialCommandGroup(
        //         new CoralIntake(coralManipulator, elevator).withTimeout(2.12) // 2.54 -
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.7))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.7))
        //         // intake and score first coral
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(4.13 + 0.6)) //10.05
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.7))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.7))
        //         //intake and score second coral
        // )));

        autoChooser.addOption("(use this) Right 2 Coral v2", new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new BlueRight2Coralv2().blueRightCoral()),
          new SequentialCommandGroup(
                new CoralIntake(coralManipulator, elevator).withTimeout(2.38)
                .andThen(new ElevatorL4(elevator, arm).withTimeout(1.65))
                .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
                .andThen(new ElevatorDown(elevator).withTimeout(1.6+0.25))
                // intake and score first coral
                .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(3.23 + 0.5))
                .andThen(new ElevatorAutoVision(elevator).withTimeout(1)) // 4.23
                .andThen(new Elevator2CoralAuto(elevator, arm).withTimeout(1.6))
                .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
                .andThen(new ArmStow(arm).withTimeout(0.1))
                .andThen(new ElevatorDown(elevator).withTimeout(1.5))
                //intake and score second coral
        )));

        // autoChooser.addOption("Left 2 Coral", new ParallelCommandGroup(
        // new WaitCommand(0.01),
        //   new SequentialCommandGroup(new BlueLeftCoral2().blueLeftCoral()),
        //   new SequentialCommandGroup(
        //         new CoralIntake(coralManipulator, elevator).withTimeout(2.38)
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.6))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.6 + 0.3))
        //         // intake and score first coral
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(4.13 + 0.1))
        //         .andThen(new ElevatorL4(elevator, arm).withTimeout(1.6))
        //         .andThen(new CoralIntake(coralManipulator, elevator).withTimeout(0.3))
        //         .andThen(new ElevatorDown(elevator).withTimeout(1.6))
        //         //intake and score second coral
        // )));

        autoChooser.addOption("Left Leave", new ParallelCommandGroup(
        new WaitCommand(0.01),
          new SequentialCommandGroup(new LeftLeave().leftLeave())
        ));

        // autoChooser.addOption("Right Leave", new ParallelCommandGroup(
        // new WaitCommand(0.01),
        //   new SequentialCommandGroup(new RightLeave().rightLeave())
        // ));
           
        //i really hope this works ^

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}