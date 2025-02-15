package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private final TalonFX arm = new TalonFX(Constants.CanIdCanivore.Algae_Arm, "Canivore");

    private final MotionMagicVoltage m_motionMagicCtrl = new MotionMagicVoltage(0);

    public Arm() {
        configureMotor();
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
            .withMotionMagicAcceleration(Constants.Arm.acceleration) // Rotations per second squared
            .withMotionMagicCruiseVelocity(Constants.Arm.velocity); // Rotations per second

        // Configure PID constants
        Slot0Configs pidConfig = new Slot0Configs()
            .withKP(Constants.Arm.kP) // Proportional gain
            .withKI(Constants.Arm.kI)  // Integral gain
            .withKD(Constants.Arm.kD) // Derivative gain
            .withKV(Constants.Arm.kV) // Velocity feedforward
            .withKA(Constants.Arm.kA); // Acceleration feedforward

        // Apply configurations
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withMotionMagic(motionMagicConfig)
            .withSlot0(pidConfig);

        arm.getConfigurator().apply(config);
    }

    public void setPosition(double positionRotations) {
        arm.setControl(m_motionMagicCtrl.withPosition(positionRotations));
    }

    public void stop() {
        arm.setControl(new DutyCycleOut(0));
    }

    public double getCurrentPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Optional: Log data for tuning
        // SignalLogger.writeDouble("Arm Position", getCurrentPosition());
        // SignalLogger.writeDouble("Arm Velocity", armMotor.getVelocity().getValue());
    }
    
}
