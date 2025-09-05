package frc.robot.subsystems.Arm;

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

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmSubsystem extends SubsystemBase {
    // סוויץ' מגנטי שנדלק בכל קומה
  
    private final MotorInterface ArmMotor;

    // גבהים לכל קומה (במטרים / יחידות אנקודר)
    private static final double[] angles = { 0, 40, 90, 120, 150, 200, 270}; 

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
    private double minangle = 0;
    private double maxangle = 270;

    public ArmSubsystem() {
        ArmMotor = new TalonMotor(Constants.ArmConfig.ArmConfig);
  
        SmartDashboard.putData("Arm", this);
        MotorCommands.showPowerCommand("Arm Power", this, ArmMotor);
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
    

    public double getHeight(){
        return ArmMotor.getCurrentPosition();
    } 

    public void setPower(double power) {
        if(power < 0 && isAtButtom()) power = 0;
        if(power > 0 && getHeight() > maxHeight) power = 0;
        ArmMotor.setDuty(power);
    }

    public void setHeight(double height) {
        if(!calibreated) return;
        height = Math.max(minHeight, Math.min(maxHeight, height));
        ArmMotor.setPositionVoltage(height, isAtButtom()? 0 :  Constants.ArmConfig.kg);
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
            ArmMotor.setEncoderPosition(minHeight);
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
            ArmMotor.setEncoderPosition(closest);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    
        builder.addStringProperty("Mode", this::getModeString, null);
        builder.addDoubleProperty("Height", this::getHeight, null);
        builder.addBooleanProperty("Calibreated", () -> calibreated, null);
        builder.addBooleanProperty("buttom", this::isAtButtom, null);
        
        builder.addDoubleProperty("Test Height", ()->ElevatorMode.Test.height, (height)->ElevatorMode.Test.height = height);
    }
}
