package frc.robot.commands.coral_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralManipulator;

public class CoralReverse extends Command{
    CoralManipulator intake;

    public CoralReverse(CoralManipulator intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setCoralVelo(0.5); 
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCoralVelo(0.0); 
    }
}