package frc.robot.commands.algae_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeManipulator;

public class AlgaeIntake extends Command{
    AlgaeManipulator intake;

    public AlgaeIntake(AlgaeManipulator intake) {
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
            intake.setAlgaeVelo(-0.7);
        } else intake.setAlgaeVelo(-0.05);
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.setAlgaeVelo(0);
    }
}