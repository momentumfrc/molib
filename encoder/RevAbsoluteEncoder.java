package first.robot.molib.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.Consumer;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;

public class RevAbsoluteEncoder implements MoEncoder.Encoder {
    public static TimeUnit VELOCITY_BASE_UNIT = Units.Seconds;

    private final Consumer<Consumer<SparkBaseConfig>> configurator;
    private final AbsoluteEncoder encoder;

    private double factor;

    public RevAbsoluteEncoder(AbsoluteEncoder encoder, Consumer<Consumer<SparkBaseConfig>> configurator) {
        this.encoder = encoder;
        this.configurator = configurator;

        setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return factor * encoder.getPosition().get();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("Cannot set position on an absolute encoder");
    }

    @Override
    public double getVelocity() {
        return factor * encoder.getVelocity().get();
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
        configurator.accept(config -> config.absoluteEncoder.inverted(inverted));
    }
}
