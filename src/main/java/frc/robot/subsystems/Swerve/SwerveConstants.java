package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveConstants {
    public static class ModuleConstants{
        public static final double Diameter = 0.1016; // in meters
        public static final double gearRatioSteer = 150.0 / 7.0;
        public static final double gearRatioDrive = 6.75;
        public static final double FL_CanCoderOffset = -82.441; // in degrees
        public static final double FR_CanCoderOffset = 150.117; // in degrees
        public static final double BR_CanCoderOffset = -60.908; // in degrees
        public static final double BL_CanCoderOffset = -80.332; // in degrees
        public static final boolean FL_SteerInverted = false;
        public static final boolean FR_SteerInverted = false;
        public static final boolean BR_SteerInverted = false;
        public static final boolean BL_SteerInverted = false;
        public static final boolean FL_DriveInverted = false;
        public static final boolean FR_DriveInverted = false;
        public static final boolean BR_DriveInverted = false;
        public static final boolean BL_DriveInverted = false;
    }

    public static class ChassisConstants{
        public static final ModuleConfig[] Config = {
            new ModuleConfig(ChassisConstants.FL_ModuleID, ChassisConstants.FL_SteerID, ChassisConstants.FL_DriveID, ChassisConstants.FL_CanCoderID, "Front_Laft", ModuleConstants.Diameter, ModuleConstants.FL_CanCoderOffset),
            new ModuleConfig(ChassisConstants.FR_ModuleID, ChassisConstants.FR_SteerID, ChassisConstants.FR_DriveID, ChassisConstants.FR_CanCoderID, "Front_Right", ModuleConstants.Diameter, ModuleConstants.FR_CanCoderOffset),
            new ModuleConfig(ChassisConstants.BR_ModuleID, ChassisConstants.BR_SteerID, ChassisConstants.BR_DriveID, ChassisConstants.BR_CanCoderID, "Back_Right", ModuleConstants.Diameter, ModuleConstants.BR_CanCoderOffset),
            new ModuleConfig(ChassisConstants.BL_ModuleID, ChassisConstants.BL_SteerID, ChassisConstants.BL_DriveID, ChassisConstants.BL_CanCoderID, "Back_Laft", ModuleConstants.Diameter, ModuleConstants.BL_CanCoderOffset)
        };
        public static final int GyroID = 0;
        public static final int FL_ModuleID = 0;
        public static final int FR_ModuleID = 0;
        public static final int BR_ModuleID = 0;
        public static final int BL_ModuleID = 0;
        public static final int FL_CanCoderID = 9;
        public static final int FR_CanCoderID = 10;
        public static final int BR_CanCoderID = 11; 
        public static final int BL_CanCoderID = 12;
        public static final int FL_SteerID = 1;
        public static final int FR_SteerID = 3;
        public static final int BR_SteerID = 5;
        public static final int BL_SteerID = 7;
        public static final int FL_DriveID = 2;
        public static final int FR_DriveID = 4;
        public static final int BR_DriveID = 6;
        public static final int BL_DriveID = 8;
        public static final double X_Y = 0.308325;
        public static final double FL_Y = 0.308325;
        public static final double FL_X = 0.308325;
        public static final double FR_Y = -0.308325;
        public static final double FR_X = 0.308325;
        public static final double BR_Y = -0.308325;
        public static final double BR_X = -0.308325;
        public static final double BL_Y = 0.308325;
        public static final double BL_X = -0.308325;
        public static final double Max_Linear_Speed = 2.0; // in meters per second
        public static final double Max_Linear_Accel = 4.0; // in meters per second squared
        public static final double Max_Rotation_Speed = 2.0; // in meters per second
        public static final ChassisSpeeds Max_Spees_PrecisionMode = new ChassisSpeeds(1.0, 1.0, 1.0); // in meters per second and radians per second
        public static final double DeadBand = 0.1;
    }
}
