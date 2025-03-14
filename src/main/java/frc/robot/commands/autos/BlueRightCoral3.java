package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlueRightCoral3 extends SubsystemBase{
    public Command BlueRightCoral3(){
        return AutoBuilder.buildAuto("blueRightCoral3");
    }
}
