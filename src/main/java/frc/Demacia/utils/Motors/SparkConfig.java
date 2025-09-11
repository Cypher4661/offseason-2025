package frc.Demacia.utils.Motors;

/** 
 * Class to hold all Spark motor configuration
 * Applicable to REV Spark Max/Flex
 *  */
public class SparkConfig extends BaseMotorConfig<SparkConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */

    double iZone = 0;

    public SparkConfig(int id, String name) {
        super(id, name);
        motorType = MotorControllerType.SparkMax;
    }

    public SparkConfig(int id, String name, SparkConfig config) {
        this(id,name);
        copyBaseFields(config);
        this.iZone = config.iZone;
    }

    public SparkConfig withIzone(double iZone) {
        this.iZone = iZone;
        return this;
    }

}