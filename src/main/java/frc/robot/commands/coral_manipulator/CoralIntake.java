package frc.robot.commands.coral_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralManipulator;

public class CoralIntake extends Command{
    CoralManipulator intake;

    public CoralIntake(CoralManipulator intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.resetLockout();
    }

    @Override
    public void execute() {
        if(!intake.isLocked()){
            intake.setCoralVelo(-0.8); 
        } else intake.setCoralVelo(0); 
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCoralVelo(0.0); 
    }
}