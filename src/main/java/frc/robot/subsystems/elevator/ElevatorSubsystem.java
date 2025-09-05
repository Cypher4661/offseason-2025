package frc.robot.subsystems.elevator;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
    // סוויץ' מגנטי שנדלק בכל קומה
    private DigitalInput buttomSwitch  = new DigitalInput(Constants.elevatorConfig.LimitSwitchID);
    private DigitalInput magenticSwitch  = new DigitalInput(Constants.elevatorConfig.MagneticLimitSwitchID);
  
    private final MotorInterface leftMotor;
    private final MotorInterface rightMotor;

    // גבהים לכל קומה (במטרים / יחידות אנקודר)
    private static final double[] magenticHeights = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}; 

    public enum ElevatorMode { Idle(0), Home(0), Intake(0.2), L1(0.3), L2(0.7), 
        L3(0.9), L4(1.2), AlgieUp(1), AlgieDown(0.4), Barge(1.2), Test(0),
        Calibreate(0);
        double height;
        private ElevatorMode(double height) {
            this.height = height;
        }
    }

    private ElevatorMode mode = ElevatorMode.Calibreate;
    private boolean calibreated = false;
    private double minHeight = 0;
    private double maxHeight = 1.2;

    public ElevatorSubsystem() {
        leftMotor = new TalonMotor(Constants.elevatorConfig.LeftMotor);
        rightMotor = new TalonMotor(Constants.elevatorConfig.RightMotor);
        ((TalonFX)rightMotor).setControl(new Follower(Constants.elevatorConfig.LeftMotor.id, true));
        
        SmartDashboard.putData("Elevetor", this);
        MotorCommands.showPowerCommand("Elevator Power", this, leftMotor);
        mode = ElevatorMode.Calibreate;
        mode.height = getHeight() + 0.05;
        setDefaultCommand(new ElevatorCommand(this));
        SmartDashboard.putString("SetMode", "test");
        SmartDashboard.putData("SetModeNow", new StartEndCommand(
        () -> setMode(SmartDashboard.getString("SetMode", "test")),
        () -> {}));
    }

    private boolean isAtButtom(){
        return !buttomSwitch.get() || getHeight() <= minHeight;
    }
    private boolean IsMagnet(){
        return !magenticSwitch.get();
    }

    public double getHeight(){
        return leftMotor.getCurrentPosition();
    } 

    public void setPower(double power) {
        if(power < 0 && isAtButtom()) power = 0;
        if(power > 0 && getHeight() > maxHeight) power = 0;
        leftMotor.setDuty(power);
    }

    public void setHeight(double height) {
        if(!calibreated) return;
        height = Math.max(minHeight, Math.min(maxHeight, height));
        leftMotor.setPositionVoltage(height, isAtButtom()? 0 :  Constants.elevatorConfig.kg);
    }

    public void setMode(ElevatorMode mode) {
        if(!calibreated) {
            this.mode = ElevatorMode.Calibreate;
        } else {
            this.mode = mode;
        }   
    }
    public void setMode(String mode) {
        try{
            setMode(ElevatorMode.valueOf(mode));
        } catch(Exception e){
            System.out.println("Invalid elevator mode: " + mode);
        }
    }

    public ElevatorMode getMode() {
        return mode;
    }

    public String getModeString() {
        return mode.toString();
    }

    @Override
    public void periodic() {
        if(buttomSwitch.get()){
            calibreated = true;
            leftMotor.setEncoderPosition(minHeight);
            setPower(0);
        } else if(calibreated && IsMagnet()) {
            double h = getHeight();
            double closest = 0;
            double error = Double.MAX_VALUE;
            for(double mh : magenticHeights){
                if(Math.abs(mh - h) < error){
                    error = Math.abs(mh - h);
                    closest = mh;
                }
            }
            leftMotor.setEncoderPosition(closest);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    
        builder.addStringProperty("Mode", this::getModeString, null);
        builder.addDoubleProperty("Height", this::getHeight, null);
        builder.addBooleanProperty("Calibreated", () -> calibreated, null);
        builder.addBooleanProperty("buttom", this::isAtButtom, null);
        builder.addBooleanProperty("magnet", this::IsMagnet, null);
        
        builder.addDoubleProperty("Test Height", ()->ElevatorMode.Test.height, (height)->ElevatorMode.Test.height = height);
    }
}
