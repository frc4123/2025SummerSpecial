package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmCoralGround extends Command{

    Arm arm;

    public ArmCoralGround(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.setPosition(Constants.Arm.coralGround);
    }
    
}
