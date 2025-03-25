package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRight3 extends SubsystemBase{
    public Command coralRight3(){
        return AutoBuilder.buildAuto("Right 3 Coral");
    }
}