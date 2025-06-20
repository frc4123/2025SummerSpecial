package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralManipulator extends SubsystemBase {
    private SparkMax intake = new SparkMax(Constants.CanIdRio.Coral_Intake, MotorType.kBrushless);
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    private CANrange canrange = new CANrange(Constants.CanIdRio.Coral_CANRange); 
    private boolean m_previousDetection; 
    private boolean m_lockedOut; 
    private boolean m_slowIntakeMode; // New state variable

    public CoralManipulator(){
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
        intake.configure(
            intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        intake.clearFaults();
    }

    public void resetLockout() {
        m_lockedOut = false;
        m_slowIntakeMode = false; // Reset slow intake mode
        m_previousDetection = isCoralDetected();  // Sync with current state
    }
    
    public void setCoralVelo(double velo) {
        intake.set(velo);
    }

    public boolean getPreviousDetection(){
        return isCoralDetected();
        //must be run in command intializer sdfit 
    }

    public boolean isLocked(){
        boolean currentDetection = isCoralDetected();
        if (currentDetection && !m_previousDetection) {
            m_lockedOut = true; 
        }
        m_previousDetection = currentDetection;
        return m_lockedOut;
    }

    public boolean isCoralDetected(){
        return canrange.getIsDetected().getValue();
    }

    public void setSlowIntakeMode(boolean enable) {
        m_slowIntakeMode = enable;
    }

    public boolean isSlowIntakeMode() {
        return m_slowIntakeMode;
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Intake Speed", -1 * intakeLeader.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Intake Temp", intakeLeader.getMotorTemperature());
    }
}