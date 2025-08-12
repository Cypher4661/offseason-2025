package frc.robot.utils;

/**
 * Abstract base class for motor configurations
 * Contains common fields and methods shared between different motor controller types
 */
public abstract class BaseMotorConfig<T extends BaseMotorConfig<T>> {
    public int id;                  // CAN bus ID
    public String name;             // Name of the motor - used for logging

    public double maxVolt = 12;     // Max Volt allowed
    public double minVolt = -12;    // Min Volts allowed
    public double maxCurrent = 40;  // Max current allowed
    public double rampUpTime = 0.3; // max power change time from 0 to full

    public boolean brake = true;    // brake/coast
    public double motorRatio = 1;   // motor to mechanism ratio
    public boolean inverted = false; // if to invert motor

    public double maxVelocity = 0;
    public double maxAcceleration = 0;
    public double maxJerk = 0;

    public closeLoopParam pid = new closeLoopParam(0, 0, 0, 0, 0, 0, 0); // close loop argument - PID + FF
    public closeLoopParam pid1 = null; // pid for slot 1
    public closeLoopParam pid2 = null; // pid for slot 2


    // enhanced ff
    public double kv2 = 0;
    public double kSin = 0;
    public double posToRad = 0;

        /** 
    * Class to hold closed loop param
    *  */
    static class closeLoopParam { // calculate volts - not -1 to 1 !!!
        double kp;  
        double ki;
        double kd;
        double ks;
        double kv;
        double ka;
        double kg;
        double kf;

        closeLoopParam(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
            this.ka = ka;
            this.kd = kd;
            this.ki = ki;
            this.kp = kp;
            this.ks = ks;
            this.kv = kv;
            this.kg = kg;
            this.kf = 0;
        }
        closeLoopParam(double kp, double ki, double kd, double kf) {
            this.ka = 0;
            this.kd = kd;
            this.ki = ki;
            this.kp = kp;
            this.ks = 0;
            this.kv = 0;
            this.kg = 0;
            this.kf = kf;
        }
    }

    /**
     * Constructor
     * @param id - CAN bus ID
     * @param name - name of motor for logging
     */
    public BaseMotorConfig(int id, String name) {
        this.id = id;
        this.name = name;
    }

    /**
     * Set voltage limits
     * @param maxVolt maximum voltage
     * @param minVolt minimum voltage
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withVolts(double maxVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = -maxVolt;
        return (T) this;
    }

    /**
     * Set brake mode
     * @param brake true for brake mode, false for coast
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withBrake(boolean brake) {
        this.brake = brake;
        return (T) this;
    }

    /**
     * Set motor inversion
     * @param invert true to invert motor direction
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withInvert(boolean invert) {
        this.inverted = invert;
        return (T) this;
    }

    /**
     * Set ramp time
     * @param rampTime time in seconds to ramp from 0 to full power
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return (T) this;
    }

    /**
     * Configure motor for linear motion (meters)
     * @param gearRatio gear ratio from motor to mechanism
     * @param circumference wheel/pulley circumference in meters
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withMeterMotor(double gearRatio, double circumference) {
        this.motorRatio = gearRatio / circumference;
        return (T) this;
    }

    /**
     * Configure motor for rotational motion (radians)
     * @param gearRatio gear ratio from motor to mechanism
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withRadiansMotor(double gearRatio) {
        this.motorRatio = gearRatio / (Math.PI * 2);
        return (T) this;
    }

    /**
     * Configure motor for rotational motion (degrees)
     * @param gearRatio gear ratio from motor to mechanism
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withDegreesMotor(double gearRatio) {
        this.motorRatio = gearRatio / 360;
        return (T) this;
    }

    /**
     * Set current limit - implementation varies by motor controller
     * @param maxCurrent maximum current in amps
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withCurrent(double maxCurrent) {
        this.maxCurrent = maxCurrent;
        return (T) this;
    }

    /**
     * Set max velocity, accelration and jerk for magic or max motion
     * @param maxCurrent maximum current in amps
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public T withVelocities(double maxVelocity, double maxAcceleration, double maxJerk) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        return (T) this;
    }

    /** 
     * Set enhanced feed forward prams
     * @param kv2
     * @param ksin
     * @param posToRad
     * @return TalonConfig
     */
    @SuppressWarnings("unchecked")
    public T withFeedForward(double kv2, double ksin, double posToRad) {
        this.kv2 = kv2;
        this.kSin = ksin;
        this.posToRad = posToRad;
        return (T)this;
    }

    /** 
     * Set pid
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return TalonConfig
     */
    public T withPID(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        return (T)withPID(1, kp, ki, kd, ks, kv, ka, kg);
    }
    /** 
     * Set pid
     * @param slot
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @param ka
     * @param kg
     * @return TalonConfig
     */
    @SuppressWarnings("unchecked")
    public T withPID(int slot, double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        switch(slot) {
            case 1:
                pid = new closeLoopParam(kp, ki, kd, ks, kv, ka, kg);
                break;
            case 2:
                pid1 = new closeLoopParam(kp, ki, kd, ks, kv, ka, kg);
                break;
            case 3:
                pid2 = new closeLoopParam(kp, ki, kd, ks, kv, ka, kg);
                break;
            default:

        }
        return (T)this;
    }
    /** 
     * Set pid
     * @param slot
     * @param kp
     * @param ki
     * @param kd
     * @param kf
     * @return TalonConfig
     */
    @SuppressWarnings("unchecked")
    public T withPID(int slot, double kp, double ki, double kd, double kf) {
        switch(slot) {
            case 1:
                pid = new closeLoopParam(kp, ki, kd, kf);
                break;
            case 2:
                pid1 = new closeLoopParam(kp, ki, kd, kf);
                break;
            case 3:
                pid2 = new closeLoopParam(kp, ki, kd, kf);
                break;
            default:

        }
        return (T)this;
    }
    /** 
     * Set pid
     * @param kp
     * @param ki
     * @param kd
     * @param kf
     * @return TalonConfig
     */
    public T withPID(double kp, double ki, double kd, double kf) {
        return (T)withPID(1, id, kp, ki, kf);
    }


    /**
     * Copy common fields from another BaseMotorConfig
     * @param other the config to copy from
     */
    protected void copyBaseFields(BaseMotorConfig<?> other) {
        this.maxVolt = other.maxVolt;
        this.minVolt = other.minVolt;
        this.maxCurrent = other.maxCurrent;
        this.rampUpTime = other.rampUpTime;
        this.brake = other.brake;
        this.motorRatio = other.motorRatio;
        this.inverted = other.inverted;
        this.maxCurrent = other.maxCurrent;
        this.kv2 = other.kv2;
        this.kSin = other.kSin;
        this.posToRad = other.posToRad;
        this.maxAcceleration = other.maxAcceleration;
        this.maxVelocity = other.maxVelocity;
        this.maxJerk = other.maxJerk;
        this.pid = other.pid;
        this.pid1 = other.pid1;
        this.pid2 = other.pid2;
   }
}