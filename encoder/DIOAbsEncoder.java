package frc.robot.molib.encoder;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * Represents a REV through-bore absolute encoder attached to a DIO port on the RIO.
 */
public class DIOAbsEncoder implements MoEncoder.Encoder {
    private static TimeUnit VELOCITY_BASE_UNIT = Units.Seconds;

    private final DutyCycleEncoder encoder;

    private double positionFactor = 1;

    public DIOAbsEncoder(int dioPort) {
        this.encoder = new DutyCycleEncoder(dioPort);
        encoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        encoder.setAssumedFrequency(1000000.0 / 1025.0);
    }

    @Override
    public double getPosition() {
        return positionFactor * encoder.get();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException(
                "operation [setPosition] not supported on encoder of type [DIOAbsEncoder]");
    }

    @Override
    public double getVelocity() {
        throw new UnsupportedOperationException(
                "operation [getVelocity] not supported on encoder of type [DIOAbsEncoder]");
    }

    @Override
    public void setInverted(boolean inverted) {
        encoder.setInverted(inverted);
    }

    @Override
    public void setPositionFactor(double factor) {
        this.positionFactor = factor;
    }

    @Override
    public TimeUnit getVelocityBaseUnit() {
        return VELOCITY_BASE_UNIT;
    }
}
