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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Camera;
import frc.robot.Constants;
import static frc.robot.Constants.*;


public class VisionSubsystem extends SubsystemBase {


    private double alpha;
    private Field2d field;

    private NetworkTableEntry cropEntry;
    private NetworkTableEntry pipeEntry;

    private boolean is3D;
    private NetworkTable Table;
    private double camToTagYaw;
    private double camToTagPitch;
    private int id;
    private double height;
    private double dist;
    public Pose2d pose;
    private double confidence;
    


    private Camera camera;
    private AHRS gyro;
    
    public VisionSubsystem(Camera camera, NetworkTableEntry cropEntry) {
        super();
        this.cropEntry = cropEntry;
        this.camera = camera;
        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        field = new Field2d();


        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();
        
        is3D = Table.getEntry("pipeline").getInteger(0) == 1;
        SmartDashboard.putData("Vision", this);
        SmartDashboard.putData("Field", field);
    }



    @Override
    public void periodic() {
        // cropEntry = Table.getEntry("crop");

        if (Table.getEntry("tv").getDouble(0.0) != 0) {
            camToTagPitch = Table.getEntry("ty").getDouble(0.0);
            camToTagYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();
            id = getTagId();
          if (id > 0 && id < TAG_HEIGHT.length) {
                pose = new Pose2d(getOriginToRobot(), getAngle());
                field.setRobotPose(pose);
                confidence = getConfidence();
            } 
        } else {
            pose = null;
        }


    }

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

    // private void crop() {
    //     double YawCrop = getYawCrop();
    //     double PitchCrop = getPitchCrop();
    //     double[] crop = { YawCrop - getCropOffset(), YawCrop + getCropOffset(), PitchCrop - getCropOffset(), PitchCrop + getCropOffset() };
    //     cropEntry.setDoubleArray(crop);
    // }

    // private double getCropOffset() {
    //     double crop = getDistFromCamera() *CROP_CONSTANT;
    //     return MathUtil.clamp(crop, MIN_CROP, MAX_CROP);
    // }



    // private double getYawCrop() {
    //     double cameraYaw = (camera.getYaw() - camToTagYaw) * 2 / camToTagYaw;
    //     return cameraYaw;
    // }

    // private double getPitchCrop() {
    //     double cameraPitch = camToTagPitch;
    //     return cameraPitch;
    // } 

    // private void stopCrop() {
    //     double[] crop = { -1, 1, -1, 1 };
    //     cropEntry.setDoubleArray(crop);
    // }
    private double getConfidence() {
        //get distance from robot to tag
        double distance = getRobotToTagVector().getNorm();
        
        //if dist is too big return 0
        if (distance > (is3D ? 20 : WORST_RELIABLE_DIST)) {
            return 0;
        }
        //if dist is close return high confidence
        if (distance <= BEST_RELIABLE_DIST) {
            return 1.0;
          }
          
        // Calculate how far we are into the falloff range (0 to 1)
        double normalizedDist = (distance - BEST_RELIABLE_DIST)
        / ((is3D ? 20 : WORST_RELIABLE_DIST) - BEST_RELIABLE_DIST);

        // higher confidence for closer distances
        return Math.pow(1 - normalizedDist, 3);

    }

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