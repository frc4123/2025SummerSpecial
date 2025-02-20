package frc.robot.commands.coral_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralManipulator;

public class CoralIntakeStop extends Command{
    CoralManipulator intake;

    public CoralIntakeStop(CoralManipulator intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setCoralVelo(0);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCoralVelo(0.0); 
    }
}