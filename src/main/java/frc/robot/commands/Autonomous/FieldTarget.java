// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static frc.robot.Constants.O_TO_TAG;
import static frc.robot.Constants.TAG_ANGLE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class FieldTarget {
    
    public enum POSITION{
        A(6, 19),
        B(7, 18), 
        C(8, 17),
        D(9, 22),
        E(10, 21),
        F(11, 20);

        private int redId;
        private int blueId;
        POSITION(int redTagID, int blueTagID){
            this.redId = redTagID;
            this.blueId = blueTagID;
        }

        public int getId() {
            return RobotContainer.isRed() ? redId:blueId;
        }
        public Pose2d getPose(){
            return new Pose2d(O_TO_TAG[getId()], TAG_ANGLE[getId()]);
        }
    }
}
