package frc.robot.commands.coral_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class CoralIntake extends Command {
    CoralManipulator intake;
    Elevator elevator;
    private boolean m_initialCoralDetected; // Track if coral was detected at command start

    public CoralIntake(CoralManipulator intake, Elevator elevator) {
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_initialCoralDetected = intake.isCoralDetected(); // Store initial coral detection state
        if (!m_initialCoralDetected) {
            // Only reset lockout if coral is not detected at the start
            intake.resetLockout();
        }
    }

    @Override
    public void execute() {
        if (m_initialCoralDetected) {
            // If coral was detected at the start, enter slow intake mode
            if (!intake.isSlowIntakeMode()) {
                intake.setSlowIntakeMode(true);
                intake.setCoralVelo(0.1); // Start slow intake
            }

            // If in slow intake mode and coral is no longer detected, stop intake
            if (intake.isSlowIntakeMode() && !intake.isCoralDetected()) {
                intake.setCoralVelo(0.0);
                intake.resetLockout();
            }
        } else {
            // If coral was not detected at the start, use normal intake behavior
            if (!intake.isLocked()) {
                double elevatorPose = elevator.getRelativePosition();
                if (elevatorPose >= 25 || elevatorPose <= 10) {
                    intake.setCoralVelo(-0.7);
                } else {
                    intake.setCoralVelo(-0.4);
                }
            } else {
                // If locked, stop intake
                intake.setCoralVelo(0.0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCoralVelo(0.0);
        if (!m_initialCoralDetected) {
            // Only reset lockout if coral was not detected at the start
            intake.resetLockout();
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when in slow intake mode and coral is no longer detected or when intake goes from not detected to detected
        return (m_initialCoralDetected && intake.isSlowIntakeMode() && !intake.isCoralDetected()) || (!m_initialCoralDetected && intake.isCoralDetected());
    }
}