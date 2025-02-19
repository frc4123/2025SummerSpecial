package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    
    public static final class CanIdCanivore { 

        public static final int Front_Left_Drive = 2;
        public static final int Front_Right_Drive = 3;
        public static final int Back_Left_Drive = 4;
        public static final int Back_Right_Drive = 5;
        // drive motors - order - start top left in clockwise rotation

        public static final int Front_Left_Turn = 6;
        public static final int Front_Right_Turn = 7;
        public static final int Back_Left_Turn = 8;
        public static final int Back_Right_Turn = 9;
        // turn motors - order - start top left in clockwise rotation

        public static final int Pigeon = 10;

        public static final int Front_Left_CANcoder = 11;
        public static final int Front_Right_CANcoder = 12;
        public static final int Back_Left_CANcoder = 13;
        public static final int Back_Right_CANcoder = 14;

        public static final int Elevator = 15;

        public static final int Algae_Arm = 17;
    }

    public static final class CanIdRio{
        public static final int Elevator_CANDi = 16; 

        public static final int Algae_Intake = 18;
        public static final int Algae_CANDi = 19;
        public static final int Algae_CANRange = 20;

        public static final int Coral_Intake = 21;
        public static final int Coral_CANRange = 22;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class Arm {
        public static final double kP = 180;
        public static final double kI = 0;
        public static final double kD = 9.5;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 4;

        public static final double velocity = 0.3;
        public static final double acceleration = 0.3;
        public static final double jerk = 0.2;

        public static final double gearRatio = 96.66;

        public static final double stowPosition = 0.34;
        public static final double reefPosition = 0.15;
        public static final double outPosition = 0;


        // public static final double stowPosition = 3.3685;
        // public static final double outPosition = 10.9;
    }

    public static final class Elevator {
        public static final double kP = 85;
        public static final double kI = 0;
        public static final double kD = 5;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 16;

        public static final double velocity = 15; //120; // max is 290 (don't go 290)
        public static final double acceleration = 15; //100;
        public static final double jerk = 15; //80; 

        public static final double gearRatio = -1;

        public static final double down = 0;
        public static final double l1 = 18;
        public static final double l2 = 30;
        public static final double l3 = 50;
        public static final double l4 = 70;
        public static final double up = 88.4;
    }

    public static final class SwerveConstants {

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final double kWheelBase = 24.75;
        public static final double kTrackWidth = Units.inchesToMeters(24.75);

        //Front Left
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.427734375);
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = false;

        public static final Distance kFrontLeftXPos = Inches.of(12.375);
        public static final Distance kFrontLeftYPos = Inches.of(12.375);

        //Front Right
        public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.444580078125);
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;

        public static final Distance kFrontRightXPos = Inches.of(12.375);
        public static final Distance kFrontRightYPos = Inches.of(-12.375);

        //Back Left
        public static final Angle kBackLeftEncoderOffset = Rotations.of(0.3447265625);
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;
    
        public static final Distance kBackLeftXPos = Inches.of(-12.375);
        public static final Distance kBackLeftYPos = Inches.of(12.375);

        //Back Right
        public static final Angle kBackRightEncoderOffset = Rotations.of(-0.191162109375);
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = false;

        public static final Distance kBackRightXPos = Inches.of(-12.375);
        public static final Distance kBackRightYPos = Inches.of(-12.375);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right
    }
}
