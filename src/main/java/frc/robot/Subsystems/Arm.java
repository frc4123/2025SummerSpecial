package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    private final MotionMagicVoltage m_motionMagicCtrl = new MotionMagicVoltage(Constants.Arm.stowPosition);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(3,20,6); // expected 0 ~ 3 for actual 0

    private final NetworkTable armStateTable = NetworkTableInstance.getDefault().getTable("ArmState");
    private final DoublePublisher absolutePublisher = armStateTable.getDoubleTopic("Absolute Position: ").publish();
    private final DoublePublisher relativePublisher = armStateTable.getDoubleTopic("Relative Position: ").publish();

    public Arm() {
        configureMotor();
        encoder.setInverted(true);
        arm.setPosition(getAbsolutePosition());
    }

    private void configureMotor() {
        // Reset factory defaults and set neutral mode
        arm.getConfigurator().apply(new TalonFXConfiguration());
        arm.setNeutralMode(NeutralModeValue.Brake);

        // Configure current limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);

        // Configure Motion Magic parameters
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
            .withMotionMagicAcceleration(Constants.Arm.acceleration)
            .withMotionMagicCruiseVelocity(Constants.Arm.velocity);

        // Configure PID constants
        Slot0Configs pidConfig = new Slot0Configs()
            .withKP(Constants.Arm.kP)
            .withKI(Constants.Arm.kI)
            .withKD(Constants.Arm.kD)
            .withKV(Constants.Arm.kV)
            .withKA(Constants.Arm.kA);

        // Apply configurations
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withMotionMagic(motionMagicConfig)
            .withSlot0(pidConfig); // Fixed typo: "@" → "0"

        arm.getConfigurator().apply(config); // Fixed method name
    }

    public void setPosition(double position) {
        arm.setControl(m_motionMagicCtrl.withPosition(position)); // Fixed syntax
    }

    public double getAbsolutePosition() {
        return encoder.get(); 
    }

    public double getRelativePosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        arm.setPosition(getAbsolutePosition());

        absolutePublisher.set(getAbsolutePosition());
        relativePublisher.set(getRelativePosition());

        armStateTable.getEntry("AbsolutePosition").setDouble(getAbsolutePosition());
        armStateTable.getEntry("RelativePosition").setDouble(getRelativePosition());

        SignalLogger.writeDouble("Arm Absolute Position", getAbsolutePosition());
        SignalLogger.writeDouble("Arm Relative Position", getRelativePosition());
    }
}