package first.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import first.molib.MoSparkConfigurator;
import first.molib.encoder.MoEncoder;
import first.molib.motune.MoTuner;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;

public class MoSparkMaxPID<Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>>
        implements MoTuner.PIDController, MoTuner.MotorFF, MoTuner.OnPopulateFinished {
    protected final Type type;
    protected final SparkBase motorController;
    protected final ClosedLoopSlot pidSlot;
    protected final SparkClosedLoopController pidController;
    protected final MoSparkConfigurator configurator;

    protected final MoEncoder<Dim, VDim> internalEncoder;

    /**
     * The last position or velocity setpoint measured in internal encoder units.
     */
    protected double lastSetpoint;

    /**
     * Constructs a MoSparkMaxPID
     * <p>
     * Note: the controller's internal encoder should be scaled to return the mechanism's position in units of rotation.
     * <p>
     * Note: we need the MoEncoder because it is solely responsible for keeping track of the encoder's internal units,
     * which are needed to calculate the units for the setpoints.
     *
     * @param type the type of PID controller
     * @param controller the motor controller
     * @param pidSlot the slot in which to save the PID constants
     */
    public MoSparkMaxPID(
            Type type,
            SparkBase controller,
            ClosedLoopSlot pidSlot,
            MoEncoder<Dim, VDim> internalEncoder,
            MoSparkConfigurator configurator) {
        this.type = type;
        this.motorController = controller;
        this.pidSlot = pidSlot;
        this.pidController = motorController.getClosedLoopController();
        this.internalEncoder = internalEncoder;
        this.configurator = configurator;
    }

    public MoSparkMaxPID(
            Type type, SparkBase controller, ClosedLoopSlot pidSlot, MoEncoder<Dim, VDim> internalEncoder) {
        this(type, controller, pidSlot, internalEncoder, MoSparkConfigurator.forSparkBase(controller));
    }

    public Type getType() {
        return type;
    }

    public ClosedLoopSlot getPidSlot() {
        return pidSlot;
    }

    @Override
    public void setP(double kP) {
        setConfigOption(config -> config.closedLoop.p(kP, pidSlot));
    }

    @Override
    public void setI(double kI) {
        setConfigOption(config -> config.closedLoop.i(kI, pidSlot));
    }

    @Override
    public void setD(double kD) {
        setConfigOption(config -> config.closedLoop.d(kD, pidSlot));
    }

    @Override
    public void setS(double kS) {
        setConfigOption(config -> config.closedLoop.feedForward.kS(kS, pidSlot));
    }

    @Override
    public void setV(double kV) {
        setConfigOption(config -> config.closedLoop.feedForward.kV(kV, pidSlot));
    }

    @Override
    public void setA(double kA) {
        setConfigOption(config -> config.closedLoop.feedForward.kA(kA, pidSlot));
    }

    public void setIZone(double iZone) {
        setConfigOption(config -> config.closedLoop.iZone(iZone, pidSlot));
    }

    public void setConfigOption(Consumer<SparkBaseConfig> op) {
        configurator.acceptWithoutPopulate(op);
    }

    public void setConfigOption(BiConsumer<SparkBaseConfig, ClosedLoopSlot> op) {
        configurator.acceptWithoutPopulate(config -> op.accept(config, pidSlot));
    }

    @Override
    public void onPopulateFinished() {
        configurator.populateConfig();
    }

    public double getLastOutput() {
        // check this, I wasn't sure which one to do. Before it was motorController.get()
        return this.motorController.getThrottle();
    }

    /**
     * Get the last position or velocity reference set on this PID controller, measured in internal encoder units.
     * @return The last setpoint of this controller
     */
    public double getSetpoint() {
        return lastSetpoint;
    }

    /**
     * Get the last position or velocity measurement of the mechanism controlled by this PID controller, measured in
     * internal encoder units.
     * @return The last measurement of the mechanism under control
     */
    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return internalEncoder.getPositionInEncoderUnits();
            case VELOCITY:
            case SMARTVELOCITY:
                return internalEncoder.getVelocityInEncoderUnits();
        }

        return 0;
    }

    public void setPositionReference(Measure<Dim> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = position.in(internalEncoder.getInternalEncoderUnits());
        pidController.setSetpoint(value, this.type.innerType, pidSlot);
        lastSetpoint = value;
    }

    public void setVelocityReference(Measure<VDim> velocity) {
        if (this.type != Type.VELOCITY && this.type != Type.SMARTVELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }
        double value = velocity.in(internalEncoder.getInternalEncoderVelocityUnits());
        pidController.setSetpoint(value, this.type.innerType, pidSlot);
        lastSetpoint = value;
    }

    public enum Type {
        POSITION(SparkBase.ControlType.kPosition),
        SMARTMOTION(SparkBase.ControlType.kMAXMotionPositionControl),
        VELOCITY(SparkBase.ControlType.kVelocity),
        SMARTVELOCITY(SparkBase.ControlType.kMAXMotionVelocityControl);

        public final SparkBase.ControlType innerType;

        private Type(SparkBase.ControlType innerType) {
            this.innerType = innerType;
        }
    }
}
