package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;

public class MiddleCoral extends SubsystemBase {

    public MiddleCoral() {}

    public Command middleCoral() {
        return AutoBuilder.buildAuto("middleCoral");
    }
}