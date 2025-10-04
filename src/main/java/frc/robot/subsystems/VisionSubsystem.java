// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Camera;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.ChassisSubsystem;

import static frc.robot.Constants.*;


public class VisionSubsystem extends SubsystemBase {

    //all of the variables that are used more than in one method in this class 
    private double alpha;
    private Field2d field;

    private NetworkTable Table;
    private double camToTagYaw;
    private double camToTagPitch;
    private int id;
    private boolean seeTag = false;
    private double height;
    private double dist;
    public Pose2d pose;
    private Translation2d robotToTag;
    private ChassisSubsystem chassis;
    private Camera camera;
    
    public VisionSubsystem(Camera camera, NetworkTableEntry cropEntry, ChassisSubsystem chassis) {
        super();
        this.chassis = chassis;
        this.camera = camera;
        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        field = new Field2d();
        SmartDashboard.putData("Vision", this);
        SmartDashboard.putData("Camera Field", field);
    }

    @Override
    public void periodic() {
        seeTag = Table.getEntry("tv").getDouble(0.0) != 0;
        //if there is'nt a tag in view, dont do anything 
        if (seeTag) {
            //get yaw and pitch from camera to tag
            camToTagPitch = Table.getEntry("ty").getDouble(0.0);
            camToTagYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();
            id = getTagId();
            if (id > 0 && id < TAG_HEIGHT.length) {
                pose = new Pose2d(getOriginToRobot(), getAngle());
                field.setRobotPose(pose);
                chassis.poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.03);
                // confidence = getConfidence();
            } 
        } else {
            pose = null;
            dist = 0;
            id = 0;
            robotToTag = null;
        }
    }

    //get the id of the tag you're looking at
    public int getTagId() {
        return (int) Table.getEntry("tid").getDouble(0.0);
    }
    //get the 2d distance from the camera to the tag
    public double getDistFromCamera() {
        if(id >= 0  && id < TAG_HEIGHT.length) {
            alpha = camToTagPitch + camera.getPitch();
            height = TAG_HEIGHT[(int) id];
            dist = Math.abs(Math.abs(height - camera.getHeight())) / Math.tan(Math.toRadians(alpha));
        } else {
            dist = 0;
        }
        return dist;
    } 
    
    //get the 2d vector from the robot to the tag
    public Translation2d getRobotToTagVector() {
        Translation2d cameraToTag = new Translation2d(getDistFromCamera(),
                Rotation2d.fromDegrees(camToTagYaw));
        robotToTag = new Translation2d(camera.getRobotToCamPosition().getX(),
                camera.getRobotToCamPosition().getY()).plus(cameraToTag);
        return robotToTag;
    }

    //get robot pose on the field
    private Translation2d getOriginToRobot() {
        //get the placement on the field of the tag you're looking at
        Translation2d origintoTag = O_TO_TAG[id];
        
        //get the height of the tag you're looking at
        height = TAG_HEIGHT[id];
        
        //   Get vector from robot to tag
        Translation2d getRobotToTagVector = getRobotToTagVector();
       
    
        Translation2d robotToTagField = getRobotToTagVector.rotateBy(getAngle());
       
        Translation2d originToRobot = origintoTag.plus(robotToTagField.rotateBy(Rotation2d.kPi));
        
    
        return originToRobot;
    }

    //get pose
    public Pose2d getPose() {
        return this.pose;
    }
    
    //check if you see a specific tag within a specific distance
    public boolean isSeeTag(int id, double distance) {
        return Table.getEntry("tid").getDouble(0.0) == id && getRobotToTagVector().getNorm() <= distance;
    }

    //check if you see any tag
    public boolean isSeeTag() {
        return Table.getEntry("tid").getDouble(0.0) > 0;
    }

    public Rotation2d getAngle() {
        return chassis.getHeading();
    }

    @Override
     public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

        // Your existing entries (fixed syntax)
        builder.addDoubleProperty("Tag ID", () -> id, null);
        builder.addDoubleProperty("Tag Dist", () -> dist, null);
        builder.addDoubleProperty("Tag Yaw", () -> camToTagYaw, null);
        builder.addDoubleProperty("Tag Pitch", () -> camToTagPitch, null);
        builder.addBooleanProperty("See Tag", () -> seeTag, null);
        builder.addDoubleProperty("Robot X", () -> pose != null ? pose.getX() : 0, null);
        builder.addDoubleProperty("Robot Y", () -> pose != null ? pose.getY() : 0, null);
        builder.addDoubleProperty("Robot to Tag X", () -> robotToTag != null ? robotToTag.getX() : 0, null);
        builder.addDoubleProperty("Robot to Tag Y", () -> robotToTag != null ? robotToTag.getY() : 0, null);
   }
}