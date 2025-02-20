package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX arm = new TalonFX(Constants.CanIdCanivore.Algae_Arm, "Canivore");
    private final DynamicMotionMagicTorqueCurrentFOC m_motionMagicCtrl = new DynamicMotionMagicTorqueCurrentFOC(Constants.Arm.stowPosition, Constants.Arm.velocity, Constants.Arm.acceleration, Constants.Arm.jerk);

    // private final NetworkTable armStateTable = NetworkTableInstance.getDefault().getTable("ArmState");


    public Arm() {
        configureMotor();
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

    public void setPosition(double position) {
        arm.setControl(m_motionMagicCtrl.withPosition(position));
    }

    public double getRelativePosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // arm.setPosition(getAbsolutePosition());
    }
}