package first.molib.encoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;
import java.util.function.Consumer;

public class RevRelativeEncoder implements MoEncoder.Encoder {
    public static final TimeUnit VELOCITY_BASE_UNIT = Units.Minute;

    private final Consumer<Consumer<SparkBaseConfig>> configurator;
    private final RelativeEncoder encoder;

    public RevRelativeEncoder(RelativeEncoder encoder, Consumer<Consumer<SparkBaseConfig>> configurator) {
        this.encoder = encoder;
        this.configurator = configurator;

        this.setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition().get();
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity().get();
    }

    @Override
    public void setPositionFactor(double factor) {
        configurator.accept(
                config -> config.encoder.positionConversionFactor(factor).velocityConversionFactor(factor));
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }

    @Override
    public void setInverted(boolean inverted) {
        configurator.accept(config -> config.encoder.inverted(inverted));
    }
}
