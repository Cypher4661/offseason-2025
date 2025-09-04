package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private DigitalInput buttomSwitch;
    private DigitalInput magneticSwitch;
    private final MotorInterface leftMotor;
    private final MotorInterface rightMotor;
    private boolean calibreated = false;

    private double testHeight = 0;

    private double magneticSwitchHeights[] = {0.3, 0.46, 0.57, 0.68, 0.75, 0.9, 1.01, 1.12};

    static enum ElevatorMode {Idle(0), Home(0), Intake(0), L2(0), L3(0), L4(0), Algie1(0), Algie2(0), Algie3(0), Test(0), Calibrate(0);
        double height = 0;
        ElevatorMode(double height) {
          this.height = height;
        }
    
    }

    private double minHeight = ElevatorMode.Home.height;
    private double maxHeight = ElevatorMode.L4.height;
    private double kg = Constants.elevatorConfig.LeftMotor.pid[0].kg();
    private ElevatorMode mode;

    public ElevatorSubsystem() {
        leftMotor = new TalonMotor(Constants.elevatorConfig.LeftMotor);
        rightMotor = new TalonMotor(Constants.elevatorConfig.RightMotor);
        ((TalonFX)rightMotor).setControl(new Follower(Constants.elevatorConfig.LeftMotor.id, true));
        buttomSwitch = new DigitalInput(Constants.elevatorConfig.buttomSwitch);
        magneticSwitch = new DigitalInput(Constants.elevatorConfig.MagneticSwitch);
        SmartDashboard.putData("Elevator", this);
        setMode(ElevatorMode.Calibrate);
        setDefaultCommand(new ElevatorCommand(this));
        leftMotor.showConfigPIDFSlotCommand(0);
        MotorCommands.showPowerCommand("Elevator Power Cmd", this, leftMotor);
    }

    public void setPower(double power) {
      if(power < 0 && (isAtButtomSwitch() || getHeight() <= minHeight))
        power = 0;
      if(power > 0 && getHeight() >= maxHeight) 
        power = 0;
      leftMotor.setDuty(power);
    }

    public double getHeight() {
      return leftMotor.getCurrentPosition();
    }
    public boolean isAtButtomSwitch() {
      return buttomSwitch.get();
    }
    public boolean isAtSwitch() {
      return magneticSwitch.get();
    }

    public void setHeight(double height) {
      height = MathUtil.clamp(height, minHeight, maxHeight);   
      leftMotor.setPositionVoltage(height, height > 0 ? kg : 0);
    }

    public ElevatorMode getMode() {
      return mode;
    }
    public void setMode(ElevatorMode mode) {
      if(calibreated) {
        this.mode = mode;
        if(mode == ElevatorMode.Calibrate) {
          mode.height = getHeight() + Constants.elevatorConfig.CalibrateUpDistance;  
        }
      } else {
        this.mode = ElevatorMode.Calibrate;
        mode.height = getHeight() + Constants.elevatorConfig.CalibrateUpDistance;
      }
    }
    public String getModeString() {
      return mode.toString();
    }
    public void setMode(String name) {
      var m = ElevatorMode.valueOf(name);
      if(m != null) {
        setMode(m);
      }
    }

    public boolean isCalibrated() {
      return calibreated;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Buttom", this::isAtButtomSwitch, null);
        builder.addBooleanProperty("Magnetic", this::isAtSwitch, null);
        builder.addDoubleProperty("Height", this::getHeight, null);
        builder.addDoubleProperty("Test Height", ()->testHeight, (height)-> {testHeight = height; ElevatorMode.Test.height = height;});
        builder.addStringProperty("Mode", this::getModeString, this::setMode);
    }

    
    @Override
    public void periodic() {
      if(isAtButtomSwitch()) {
          leftMotor.setEncoderPosition(ElevatorMode.Home.height);
          calibreated = true;
      }
      if(calibreated) {
        if(isAtSwitch()) {
          double height = getHeight();
          double best = 0;
          double error = Double.MAX_VALUE;
          for(double h : magneticSwitchHeights) {
            double t = Math.abs(h-height);
            if(t < error) {
              best = h;
              error = t;
            }
          }
          leftMotor.setEncoderPosition(best);
          LogManager.log("elevator at magentic - set height to " + best + " corrected " + error);
        }
      }
    }


}
