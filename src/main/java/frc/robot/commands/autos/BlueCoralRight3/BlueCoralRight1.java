package frc.robot.commands.autos.BlueCoralRight3;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlueCoralRight1 extends SubsystemBase{
    public Command blueRightCoral(){
        return AutoBuilder.buildAuto("BlueRightCoral3 1");
    }
}
