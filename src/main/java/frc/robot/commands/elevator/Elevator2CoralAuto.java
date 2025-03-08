package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Elevator2CoralAuto extends Command{

    Elevator elevator;
    Arm arm;

    public Elevator2CoralAuto(Elevator elevator, Arm arm){
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator);
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.setPosition(Constants.Arm.upPosition);
        elevator.setPosition(Constants.Elevator.l4auto);
    }
    
}
