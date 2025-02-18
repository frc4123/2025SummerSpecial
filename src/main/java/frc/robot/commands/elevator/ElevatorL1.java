package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorL1 extends Command{

    Elevator elevator;

    public ElevatorL1(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute(){
        elevator.setPosition(Constants.Elevator.l1);
    }
    
}
