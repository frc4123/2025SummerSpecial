package frc.robot.commands.coral_manipulator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class CheckIntake extends Command{
    CoralManipulator intake;
    Elevator elevator;
    boolean isCoralIn;

    public CheckIntake(CoralManipulator intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.resetLockout();
        isCoralIn = intake.getPreviousDetection();
    }

    @Override
    public void execute() {
        if(!isCoralIn){
            if(!intake.isLocked()){
                double elevatorPose = elevator.getRelativePosition();
                if(elevatorPose >= 25 || elevatorPose <= 10){
                    intake.setCoralVelo(-0.6);
                } else intake.setCoralVelo(-0.4); 

            } else intake.setCoralVelo(0); 
        } else end(true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCoralVelo(0.0); 
        intake.resetLockout();
    }
}