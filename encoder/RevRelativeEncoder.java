package first.robot.molib.encoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.Consumer;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Units;

public class RevRelativeEncoder implements MoEncoder.Encoder {
    public static final TimeUnit VELOCITY_BASE_UNIT = Units.Minute;

    private final Consumer<Consumer<SparkBaseConfig>> configurator;
    private final RelativeEncoder encoder;

    private double factor;

    public RevRelativeEncoder(RelativeEncoder encoder, Consumer<Consumer<SparkBaseConfig>> configurator) {
        this.encoder = encoder;
        this.configurator = configurator;

        this.setPositionFactor(1);
    }

    @Override
    public double getPosition() {
        return factor * encoder.getPosition().get();
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position / factor);
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
        configurator.accept(config -> config.encoder.inverted(inverted));
    }
}
