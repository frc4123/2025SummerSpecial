package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DiegoAuto extends SubsystemBase{
    public Command diegoAuto(){
        return AutoBuilder.buildAuto("Diego Right 3 Coral");
    }
}
