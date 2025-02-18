package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX arm = new TalonFX(Constants.CanIdCanivore.Algae_Arm, "Canivore");
    private final DynamicMotionMagicTorqueCurrentFOC m_motionMagicCtrl = new DynamicMotionMagicTorqueCurrentFOC(Constants.Arm.stowPosition, Constants.Arm.velocity, Constants.Arm.acceleration, Constants.Arm.jerk);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(3); // expected 0 ~ 3 for actual 0


    private final NetworkTable armStateTable = NetworkTableInstance.getDefault().getTable("ArmState");
    private final DoublePublisher absolutePublisher = armStateTable.getDoubleTopic("Absolute Position: ").publish();
    private final DoublePublisher relativePublisher = armStateTable.getDoubleTopic("Relative Position: ").publish();

    private double offset;

    public Arm() {
        configureMotor();
        encoder.setInverted(true);
        offset = encoder.get();
        //encoder.setDutyCycleRange(1.0/1024.0, 1023.0/1024.0);
    }

    private void configureMotor() {
        arm.setNeutralMode(NeutralModeValue.Brake);


        // Configure PID constants
        Slot0Configs pidConfig = new Slot0Configs()
            .withKP(Constants.Arm.kP)
            .withKI(Constants.Arm.kI)
            .withKD(Constants.Arm.kD)
            .withKV(Constants.Arm.kV)
            .withKA(Constants.Arm.kA);   

        arm.getConfigurator().apply(pidConfig);
    }

    public void syncEncoderPosition() {
        arm.setPosition(getAbsolutePosition());
    }

    public void setPosition(double position) {
        arm.setControl(m_motionMagicCtrl.withPosition(position));
    }

    public double getAbsolutePosition() {
        return (encoder.get() - offset) * Constants.Arm.gearRatio; 
    }

    public double getRelativePosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // arm.setPosition(getAbsolutePosition());

        absolutePublisher.set(getAbsolutePosition());
        relativePublisher.set(getRelativePosition());

        armStateTable.getEntry("AbsolutePosition").setDouble(getAbsolutePosition());
        armStateTable.getEntry("RelativePosition").setDouble(getRelativePosition());
    }
}