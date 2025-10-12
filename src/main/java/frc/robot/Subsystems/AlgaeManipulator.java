package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AlgaeManipulator extends SubsystemBase{

    private TalonFX intake = new TalonFX(Constants.CanIdCanivore.Algae_Intake, "Canivore");

    private CANrange canrange = new CANrange(Constants.CanIdCanivore.Algae_CANRange, "Canivore"); 
    private boolean m_previousDetection; 
    private boolean m_lockedOut; 

    public AlgaeManipulator(){
        
        intake.setNeutralMode(NeutralModeValue.Brake);
        //STATORCURRENT LIMITS
    }

    public double getCurrentDistance(){
        return canrange.getDistance().getValueAsDouble();
    }

    public boolean isAlgaeDetected(){
        return canrange.getIsDetected().getValue();
    }

    public void resetLockout() {
        m_lockedOut = false;
        m_previousDetection = isAlgaeDetected();  // Sync with current state
    }
    
    public void setAlgaeVelo(double velo) {
        intake.set(velo);
        //CHECK FORWARDS / BACKWARDS
    }

    public boolean isLocked(){
        
        boolean currentDetection = isAlgaeDetected();
        if (currentDetection && !m_previousDetection) {
            m_lockedOut = true; 
        }
        m_previousDetection = currentDetection;
        return m_lockedOut;
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Intake Speed", -1 * intakeLeader.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Intake Temp", intakeLeader.getMotorTemperature());
    }
}