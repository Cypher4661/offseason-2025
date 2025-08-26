package frc.robot.subsystems.Swerve;

public class SwerveConstants {
    public static class ModuleConstants{
        public static final double Diameter = 0.1016; // in meters
        public static final double gearRatioSteer = 150.0 / 7.0;
        public static final double gearRatioDrive = 6.75;
        public static final double FL_CanCoderOffset = 8.0; // in degrees
        public static final double FR_CanCoderOffset = 8.0; // in degrees
        public static final double BR_CanCoderOffset = 8.0; // in degrees
        public static final double BL_CanCoderOffset = 8.0; // in degrees
        public static final double Steer_KP = 0.0;
        public static final double Steer_KI = 0.0;
        public static final double Steer_KD = 0.0;
        public static final double Steer_KS = 0.0;
        public static final double Steer_KV = 0.0;
        public static final double Steer_KA = 0.0;
        public static final double Steer_KG = 0.0;
        public static final double Drive_KP = 0.0;
        public static final double Drive_KI = 0.0;
        public static final double Drive_KD = 0.0;
        public static final double Drive_KS = 0.0;
        public static final double Drive_KV = 0.0;
        public static final double Drive_KA = 0.0;
        public static final double Drive_KG = 0.0;
    }

    public static class ChasissConstants{
        public static final ModuleConfig[] Config = {
            new ModuleConfig(ChasissConstants.FL_ModuleID, ChasissConstants.FL_SteerID, ChasissConstants.FL_DriveID, ChasissConstants.FL_CanCoderID, "Front_Laft", ModuleConstants.Diameter, ModuleConstants.FL_CanCoderOffset),
            new ModuleConfig(ChasissConstants.FR_ModuleID, ChasissConstants.FR_SteerID, ChasissConstants.FR_DriveID, ChasissConstants.FR_CanCoderID, "Front_Right", ModuleConstants.Diameter, ModuleConstants.FR_CanCoderOffset),
            new ModuleConfig(ChasissConstants.BR_ModuleID, ChasissConstants.BR_SteerID, ChasissConstants.BR_DriveID, ChasissConstants.BR_CanCoderID, "Back_Right", ModuleConstants.Diameter, ModuleConstants.BR_CanCoderOffset),
            new ModuleConfig(ChasissConstants.BL_ModuleID, ChasissConstants.BL_SteerID, ChasissConstants.BL_DriveID, ChasissConstants.BL_CanCoderID, "Back_Laft", ModuleConstants.Diameter, ModuleConstants.BL_CanCoderOffset)
        };
        public static final int FL_ModuleID = 1;
        public static final int FR_ModuleID = 2;
        public static final int BR_ModuleID = 3;
        public static final int BL_ModuleID = 4;
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
        
    }
}
