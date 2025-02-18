package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator = new TalonFX(Constants.CanIdCanivore.Elevator, "Canivore");
    private final DynamicMotionMagicTorqueCurrentFOC m_motionMagicCtrl = new DynamicMotionMagicTorqueCurrentFOC(Constants.Elevator.down, Constants.Elevator.velocity, Constants.Elevator.acceleration, Constants.Elevator.jerk);

    private final NetworkTable armStateTable = NetworkTableInstance.getDefault().getTable("Elevator State");
    private final DoublePublisher relativePublisher = armStateTable.getDoubleTopic("Elevator Relative Position: ").publish();

    public Elevator() {
        configureMotor();
    }

    private void configureMotor() {
        elevator.setNeutralMode(NeutralModeValue.Brake);

        // Configure PID constants
        Slot0Configs pidConfig = new Slot0Configs()
            .withKP(Constants.Elevator.kP)
            .withKI(Constants.Elevator.kI)
            .withKD(Constants.Elevator.kD)
            .withKV(Constants.Elevator.kV)
            .withKA(Constants.Elevator.kA)
            .withKG(Constants.Elevator.kG);   

        elevator.getConfigurator().apply(pidConfig);
    }

    public void setPosition(double position) {
        elevator.setControl(m_motionMagicCtrl.withPosition(position));
    }

    public double getRelativePosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getElevatorSwerveReduction(){
        double percentOfMaxHeight = getRelativePosition() / Constants.Elevator.up;
        double elevatorReductionPercent = Math.abs(percentOfMaxHeight - 1);
        return elevatorReductionPercent <= 0.1 ? 0.1 : elevatorReductionPercent;
    }

    @Override
    public void periodic() {
        relativePublisher.set(getRelativePosition());
        armStateTable.getEntry("Elevator RelativePosition").setDouble(getRelativePosition());
    }
}