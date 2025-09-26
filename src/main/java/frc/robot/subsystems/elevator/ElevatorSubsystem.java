package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Demacia.utils.Log.LogManager;
import frc.Demacia.utils.Motors.MotorCommands;
import frc.Demacia.utils.Motors.MotorInterface;
import frc.Demacia.utils.Motors.TalonMotor;
import frc.Demacia.utils.Sensors.Cancoder;
import frc.robot.Constants;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {

    boolean haveArm = false; // if arm motor and encoder exists
    boolean elevatorOnly = true; // in testing - only move the elevator and not the arm
    

    // Magnetic sensor heights
    private static final double[] magenticHeights = { 0.1, 0.15, 0.22, 0.29, 0.35, 0.40, 0.5, 0.6}; 
    private DigitalInput magenticSwitch  = new DigitalInput(Constants.elevatorConfig.MagneticLimitSwitchID);
    private DigitalInput buttomSwitch  = new DigitalInput(Constants.elevatorConfig.LimitSwitchID);
  
    private final MotorInterface leftMotor;
    private final MotorInterface rightMotor;
    private final MotorInterface armMotor;
    private final Cancoder cancoder;            
    private double armOffset = Constants.Arm.ARM_CANCODER_OFFSET; // used to set the offset from Elastic
    SendableChooser<ElevatorMode> modeChooser;
    double encoderOffset;

    
    public enum ElevatorMode { 
        Idle(0, -70), 
        Home(0, -70), 
        Intake(0.2, -50), 
        L1(0.4, -20), 
        L2(0.2, 20), 
        L3(0.3, 50), 
        L4(0.6,40), 
        AlgieUp(0.3,60), 
        AlgieDown(0.22, 30), 
        Barge(0.6, 90), 
        Test(0,0),
        Calibreate(0, -70);
        double height;
        double angle;
        private ElevatorMode(double height, double angle) {this.height = height; this.angle = angle; }
    }

    private ElevatorMode mode = ElevatorMode.Calibreate;
    private boolean calibreated = false;
    private double minHeight = 0;
    private double maxHeight = 0.8;
    private double minAngle = -70;
    private double  maxAngle = 90;

    public ElevatorSubsystem() {
        LogManager.log("ElevatorSubsystem started");
        leftMotor = new TalonMotor(Constants.elevatorConfig.LeftMotor);
        rightMotor = new TalonMotor(Constants.elevatorConfig.RightMotor);
        ((TalonFX)rightMotor).setControl(new Follower(Constants.elevatorConfig.LeftMotor.id, true));

        // start mode
        mode = ElevatorMode.Idle;
        ElevatorMode.Calibreate.height = getHeight() + 0.05; // set the height so that the elevatro will move up 5 cm before going doen to the sensor
        calibreated = true;
        encoderOffset = minHeight - leftMotor.getCurrentPosition();

        if(haveArm) {
            armMotor = new TalonMotor(Constants.Arm.ARM_CONFIG);
            cancoder = new Cancoder(Constants.Arm.ARM_CANCODER);
            calibrateFromCancoder();
            armMotor.showConfigPIDFSlotCommand(0);
            armMotor.showConfigMotionVelocitiesCommand();
            MotorCommands.showPowerCommand("Arm Power", this, armMotor);
            MotorCommands.showAngleCommand("Arm Angle cmd", this, armMotor);
        } else {
            armMotor = null;
            cancoder = null;
        }
       
        // Elastic - sysid/testing data
        leftMotor.showConfigPIDFSlotCommand(0);
        leftMotor.showConfigMotionVelocitiesCommand();
        MotorCommands.showPowerCommand("Elevator Power", this, leftMotor);
        MotorCommands.showPositionCommand("Elevator Position", this, leftMotor);
        SmartDashboard.putData("Elecator Command",new ElevatorCommand(this));
        // set the default command
        // setDefaultCommand(new ElevatorCommand(this));
        SmartDashboard.putData("Elevetor", this);
        encoderOffset = 0;
    }
    

    private boolean isAtButtom(){
        return !buttomSwitch.get() || getHeight() <= minHeight;
    }

    private boolean buttomSwitch() {
        return !buttomSwitch.get();
    }

    private boolean IsAtMagnet(){
        return !magenticSwitch.get();
    }

    public double getHeight(){
        return leftMotor.getCurrentPosition() + encoderOffset;
    } 

    public double getAngle() {
        return haveArm ? armMotor.getCurrentAngle() : 0;
    }
    public double getAbsAngle(){
        return haveArm ? cancoder.getCurrentAbsPosition() : 0;
    }
    

    public void setArmPower(double percent) {
        if(haveArm) {
            double angle = getAngle();
            if((percent < 0 && angle <= minAngle) || (percent > 0 && angle >= maxAngle)) {
                percent = 0;
            }
            armMotor.setDuty(percent);
        }
    }

    public void stopArm() {
        setArmPower(0);
    }
    
    

    public void calibrateFromCancoder() {
        if(haveArm) {
            double absDegrees = cancoder.getCurrentAbsPosition();
            armMotor.setEncoderPosition(absDegrees - armOffset);
        }
    }

    public void setAngle(double targetDeg) {
        if(haveArm) {
            armMotor.setMotion(MathUtil.clamp(targetDeg, minAngle, maxAngle), Constants.Arm.kG * Math.cos(Math.toRadians(getAngle())));
        }
    }

    public void setElvPower(double power) {
        if(power < 0 && isAtButtom()) power = 0;
        if(power > 0 && getHeight() > maxHeight) power = 0;
        leftMotor.setDuty(power);
    }

    public void setHeight(double height) {
        if(!calibreated) return;
        height = Math.max(minHeight, Math.min(maxHeight, height));
        leftMotor.setPositionVoltage(height-encoderOffset, isAtButtom()? 0 :  Constants.elevatorConfig.KG);
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
        if(buttomSwitch()){
            calibreated = true;
            encoderOffset = minHeight - leftMotor.getCurrentPosition();
            setElvPower(0);
        } else if(calibreated && IsAtMagnet()) {
            double h = getHeight();
            double closest = 0;
            double error = Double.MAX_VALUE;
            for(double mh : magenticHeights){
                if(Math.abs(mh - h) < error){
                    error = Math.abs(mh - h);
                    closest = mh;
                }
            }
            LogManager.log("Elevator magnet at height " + closest + " corrected " + (closest - h));
            encoderOffset = closest - leftMotor.getCurrentPosition();
        }
    }

    private void addModeChooser() {
        modeChooser = new SendableChooser<>();
        modeChooser.setDefaultOption(ElevatorMode.Idle.name(), ElevatorMode.Idle);
        for(ElevatorMode mode : ElevatorMode.values()) {
            if(mode != ElevatorMode.Idle)
                modeChooser.addOption(mode.name(), mode);
        }
        modeChooser.onChange((mode)->setMode(mode));
        SmartDashboard.putData("Mode",modeChooser);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        addModeChooser();
        builder.addStringProperty("Mode", this::getModeString, null);
        builder.addDoubleProperty("Height", this::getHeight, null);
        builder.addBooleanProperty("Calibreated", () -> calibreated, null);
        builder.addBooleanProperty("buttom", this::buttomSwitch, null);
        builder.addBooleanProperty("magnet", this::IsAtMagnet, null);
        if(haveArm) {
            builder.addDoubleProperty("Angle", this::getAngle, null);
            builder.addDoubleProperty("AbsAngle", this::getAbsAngle, null);
            builder.addBooleanProperty("Elevator Only", ()->elevatorOnly, (b)->elevatorOnly = b);
            builder.addDoubleProperty("Arm Offset", ()->armOffset, (o)->{armOffset = 0; calibrateFromCancoder();});
        }
        
        builder.addDoubleProperty("Test Height", ()->ElevatorMode.Test.height, (height)->ElevatorMode.Test.height = height);
        builder.addDoubleProperty("Test Angle", ()->ElevatorMode.Test.angle, (angle)->ElevatorMode.Test.angle = angle);
    }
}
