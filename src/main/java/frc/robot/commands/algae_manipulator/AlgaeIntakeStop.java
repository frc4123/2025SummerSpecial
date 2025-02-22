package frc.robot.commands.algae_manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulator;

public class AlgaeIntakeStop extends Command{
    AlgaeManipulator intake;

    public AlgaeIntakeStop(AlgaeManipulator intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(!intake.isLocked()){
            intake.setAlgaeVelo(0);
        } else intake.setAlgaeVelo(-0.12);
        
    }
}