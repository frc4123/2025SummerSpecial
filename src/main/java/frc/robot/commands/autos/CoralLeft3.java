package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralLeft3 extends SubsystemBase{
    public Command coralLeft3(){
        return AutoBuilder.buildAuto("Left 3 Coral");
    }
}