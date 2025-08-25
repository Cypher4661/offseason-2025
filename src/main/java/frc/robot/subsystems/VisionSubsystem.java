// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Camera;
import frc.robot.Camera.CameraType;
import static frc.robot.Constants.*;


public class VisionSubsystem extends SubsystemBase {


    private Translation2d robotToTag;
    private Translation2d cameraToTag;
    private double alpha;
    private Field2d field;


    private NetworkTable Table;
    private double camToTagYaw;
    private double camToTagPitch;
    private double id;
    private double height;
    private double dist;
    private Translation2d originToRobot;
    private Translation2d origintoTag;
    private Translation2d robotToTagRR;
    public Pose2d pose;

    private Supplier<Rotation2d> getRobotAngle;

    private Camera camera;

    private double tagID = 0;
    private double Yaw3d;
    private Rotation2d yaw3dRotation2d;


    public VisionSubsystem(Supplier<Rotation2d> getRobotAngle, Camera camera) {
        this.getRobotAngle = getRobotAngle;
        this.camera = camera;
        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        field = new Field2d();
    }

    public Field2d getField2d() {
        return field;
    }

    @Override
    public void periodic() {
        camToTagPitch = Table.getEntry("ty").getDouble(0.0);
        camToTagYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();
        id = getTagId();

        if (Table.getEntry("tv").getDouble(0.0) != 0) {
            if (id > 0 && id < TAG_HEIGHT.length) {
                pose = new Pose2d(getOriginToRobot(), getRobotAngle.get());
                field.setRobotPose(pose);
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
        if (camera.getCameraType() == CameraType.REEF) {
            dist = (Math.abs(height - camera.getHeight())) * Math.tan(Math.toRadians(alpha));
            dist = dist / Math.cos(Math.toRadians(camToTagYaw));
            return Math.abs(dist);
        }
        dist = (Math.abs(height - camera.getHeight())) / Math.tan(Math.toRadians(alpha));
        dist = dist / Math.cos(Math.toRadians(camToTagYaw));
        return Math.abs(dist);
    }

    public Translation2d getRobotToTagRR() {
        cameraToTag = new Translation2d(getDistFromCamera(),
                Rotation2d.fromDegrees(camToTagYaw));
        robotToTag = new Translation2d(camera.getRobotToCamPosition().getX(),
                camera.getRobotToCamPosition().getY()).plus(cameraToTag);
        return robotToTag;
    }

    public Translation2d getOriginToRobot() {
        origintoTag = O_TO_TAG[(int) this.id == -1 ? 0 : (int) this.id];
        height = TAG_HEIGHT[(int) this.id];
        if (origintoTag != null) {
            robotToTagRR = getRobotToTagRR();
            Translation2d robotToTagFC = robotToTagRR.rotateBy(getRobotAngle.get());
            originToRobot = origintoTag.plus(robotToTagFC.rotateBy(Rotation2d.kPi));
            return originToRobot;
        }
        return new Translation2d();
    }

    public Pose2d getPose() {
        return this.pose;
    }

    public Rotation2d getRobotAngle() {
        try {
            Yaw3d = Table.getEntry("camerapose_targetspace").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0})[4];
            tagID = Table.getEntry("tid").getDouble(0.0);
            yaw3dRotation2d = Rotation2d.fromDegrees(Yaw3d)
                    .rotateBy(Rotation2d.fromDegrees(camera.getYaw()))
                    .rotateBy(TAG_ANGLE[(int) tagID])
                    .rotateBy(Rotation2d.fromDegrees(180));
            return yaw3dRotation2d;
        } catch (Exception E) {
            return getRobotAngle();
        }
    }

    public boolean isSeeTag(int id, double distance) {
        return Table.getEntry("tid").getDouble(0.0) == id && getRobotToTagRR().getNorm() <= distance;
    }

    public boolean isSeeTag() {
        return Table.getEntry("tid").getDouble(0.0) > 0;
    }

    public double getAngle() {
        return Table.getEntry("botpose").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0})[5];
    }
}