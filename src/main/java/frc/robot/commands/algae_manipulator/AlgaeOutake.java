package frc.robot.commands.algae_manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulator;

public class AlgaeOutake extends Command{
    AlgaeManipulator outake;

    public AlgaeOutake(AlgaeManipulator outake) {
        this.outake = outake;
        addRequirements(outake);
    }

    @Override
    public void execute() {
        outake.setAlgaeVelo(0.7);
    }

    @Override
    public void end(boolean interrupted) {
        outake.setAlgaeVelo(0.0); 
    }
}