// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Demacia.utils.Motors.MotorInterface;

/** Add your docs here. */
public class ElevatorOffset {
    DigitalInput buttomSwitch;
    DigitalInput magnetSwitch;
    boolean lastButtom;
    boolean lastMagnet;
    double offsetUp = 0;
    double offsetDown = 0;
    MotorInterface motor;

    public ElevatorOffset(DigitalInput buttomSwitch, DigitalInput magnetSwitch, MotorInterface motor) {
        this.buttomSwitch = buttomSwitch;
        this.magnetSwitch = magnetSwitch;
        this.motor = motor;
        lastButtom = buttom();
        lastMagnet = magnet();
    }

    private boolean buttom() {
        return buttomSwitch.get();
    }

    private boolean magnet() {
        return !magnetSwitch.get();
    }

    private boolean up() {
        return motor.getCurrentVelocity() >= 0;
    }

    private double height() {
        return motor.getCurrentPosition();
    }

    public void process() {
        if(buttom() != lastButtom) {
            if(up()) {
                SmartDashboard.putNumber("up buttom height", height());
            } else {
                SmartDashboard.putNumber("down buttom height", height());
            }
            lastButtom = !lastButtom;
        }
        if(magnet() != lastMagnet) {
            if(up()) {
                SmartDashboard.putNumber("up magnet height", height());
            } else {
                SmartDashboard.putNumber("down magent height", height());
            }
            lastMagnet = !lastMagnet;
        }
    }

}
