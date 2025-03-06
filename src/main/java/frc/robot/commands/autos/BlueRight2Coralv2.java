package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlueRight2Coralv2 extends SubsystemBase{
    public Command blueRightCoral(){
        return AutoBuilder.buildAuto("blueRightCoral_2_2.0");
    }
}
