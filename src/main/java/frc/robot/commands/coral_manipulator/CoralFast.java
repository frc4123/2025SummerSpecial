package frc.robot.commands.coral_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class CoralFast extends Command{
    CoralManipulator intake;
    Elevator elevator;

    public CoralFast(CoralManipulator intake, Elevator elevator) {
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.resetLockout();
    }

    @Override
    public void execute() {
        if(!intake.isLocked()){
            double elevatorPose = elevator.getRelativePosition();
            if(elevatorPose >= 25 || elevatorPose <= 10){
                intake.setCoralVelo(-0.6);
            } else intake.setCoralVelo(-0.9); 

        } else intake.setCoralVelo(0); 
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCoralVelo(0.0); 
    }
}