package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LeftLeave extends SubsystemBase{
    public Command leftLeave(){
        return AutoBuilder.buildAuto("Left Leave");
    }
}
