package first.robot.molib.encoder;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.Consumer;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;

public class RevAnalogSensorEncoder implements MoEncoder.Encoder {
    public static TimeUnit VELOCITY_BASE_UNIT = Units.Seconds;

    private final Consumer<Consumer<SparkBaseConfig>> configurator;
    private final SparkAnalogSensor sensor;

    private double factor;

    public RevAnalogSensorEncoder(SparkAnalogSensor sparkAnalog, Consumer<Consumer<SparkBaseConfig>> configurator) {
        this.sensor = sparkAnalog;
        this.configurator = configurator;

        this.setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return factor * sensor.getPosition().get();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("Cannot set position on an absolute encoder");
    }

    @Override
    public double getVelocity() {
        return factor * sensor.getVelocity().get();
    }

    @Override
    public void setPositionFactor(double factor) {
        this.factor = factor;
    }

    @Override
    public double getPositionFactor() {
        return factor;
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        configurator.accept(config -> config.analogSensor.inverted(inverted));
    }
}
