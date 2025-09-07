// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Camera;
import frc.robot.Constants;
import static frc.robot.Constants.*;


public class VisionSubsystem extends SubsystemBase {


    private double alpha;
    private Field2d field;


    private NetworkTable Table;
    private double camToTagYaw;
    private double camToTagPitch;
    private int id;
    private double height;
    private double dist;
    public Pose2d pose;


    private Camera camera;
    private AHRS gyro;
    
    private final SendableChooser<String> allianceChooser = new SendableChooser<>();

    private double currentGyroOffset;

    public VisionSubsystem(Camera camera) {
        super();
        this.camera = camera;
        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        field = new Field2d();


        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();
        SmartDashboard.putData("Vision", this);
        SmartDashboard.putData("Field", field);
        
        allianceChooser.setDefaultOption("Blue", "blue");
        allianceChooser.addOption("Red", "red");
        allianceChooser.addOption("Auto (DS Alliance)", "auto");
        SmartDashboard.putData("Alliance Color", allianceChooser);
    }



    @Override
    public void periodic() {
        
        if (Table.getEntry("tv").getDouble(0.0) != 0) {
            camToTagPitch = Table.getEntry("ty").getDouble(0.0);
            camToTagYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();
            id = getTagId();
          if (id > 0 && id < TAG_HEIGHT.length) {
                pose = new Pose2d(getOriginToRobot(), getAngle());
                field.setRobotPose(pose);
            } 
        } else {
            pose = null;
        }


    }

    // public void updateGyroOffset() {
    //     String alliance = getAllianceColor();
    //     if ("red".equalsIgnoreCase(alliance)) {
    //         currentGyroOffset = Constants.gyroOffsetRed;
    //     } else {
    //         currentGyroOffset = Constants.gyroOffsetBlue;
    //     }
    // }

    public int getTagId() {
        return (int) Table.getEntry("tid").getDouble(0.0);
    }

    public double getDistFromCamera() {
        alpha = camToTagPitch + camera.getPitch();
        height = TAG_HEIGHT[(int) id];

        alpha = camToTagPitch + camera.getPitch();
        dist = (Math.abs(height - camera.getHeight())) / Math.tan(Math.toRadians(alpha));
        
        return Math.abs(dist);
    } 

    public Translation2d getRobotToTagVector() {
        Translation2d cameraToTag = new Translation2d(getDistFromCamera(),
                Rotation2d.fromDegrees(camToTagYaw));
        Translation2d robotToTag = new Translation2d(camera.getRobotToCamPosition().getX(),
                camera.getRobotToCamPosition().getY()).plus(cameraToTag);
        return robotToTag;
    }

    public Translation2d getCameraToTag() {
        return new Translation2d(getDistFromCamera(),
            Rotation2d.fromDegrees(camToTagYaw));
    }

    private Translation2d getOriginToRobot() {

        Translation2d origintoTag = O_TO_TAG[id];
    
        height = TAG_HEIGHT[id];
        //   Get vector from robot to tag
        Translation2d getRobotToTagVector = getRobotToTagVector();
    
        Translation2d robotToTagField = getRobotToTagVector.rotateBy(getAngle());
        Translation2d originToRobot = origintoTag.plus(robotToTagField.rotateBy(Rotation2d.kPi));
    
        return originToRobot;
    }


    public Pose2d getPose() {
        return this.pose;
    }

    public boolean isSeeTag(int id, double distance) {
        return Table.getEntry("tid").getDouble(0.0) == id && getRobotToTagVector().getNorm() <= distance;
    }

    public boolean isSeeTag() {
        return Table.getEntry("tid").getDouble(0.0) > 0;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-gyro.getYaw() + Constants.gyroOffset);
    }

    // public String getAllianceColor() {
    //     String selected = allianceChooser.getSelected();
    //     if ("red".equalsIgnoreCase(selected)) return "red";
    //     if ("blue".equalsIgnoreCase(selected)) return "blue";

    //     // Auto mode = take from DS
    //     return DriverStation.getAlliance().isPresent() &&
    //            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    //            ? "red" : "blue";
    // }

   @Override
     public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

        // Your existing entries (fixed syntax)
        builder.addDoubleProperty("Tag ID", () -> getTagId(), null);
        builder.addDoubleProperty("Tag Dist", () -> getDistFromCamera(), null);
        builder.addDoubleProperty("Tag Yaw", () -> camToTagYaw, null);
        builder.addDoubleProperty("Tag Pitch", () -> camToTagPitch, null);
        builder.addBooleanProperty("See Tag", () -> isSeeTag(), null);
        builder.addDoubleProperty("Robot X", () -> getOriginToRobot().getX(), null);
        builder.addDoubleProperty("Robot Y", () -> getOriginToRobot().getY(), null);
        builder.addDoubleProperty("Robot to Tag X", () -> getRobotToTagVector().getX(), null);
        builder.addDoubleProperty("Robot to Tag Y", () -> getRobotToTagVector().getY(), null);

        
        // Add gyro diagnostics
        builder.addBooleanProperty("Gyro Connected", () -> gyro.isConnected(), null);
        builder.addBooleanProperty("Gyro Calibrating", () -> gyro.isCalibrating(), null);
        builder.addDoubleProperty("Gyro Yaw", () -> gyro.getYaw(), null);

        // builder.addStringProperty("Alliance Color", () -> getAllianceColor(), null);
   }
}
