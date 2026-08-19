package first.molib.pid;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import first.molib.encoder.TalonFxEncoder;
import first.molib.motune.MoTuner;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;

/**
 * A MoTalonFxPID specialized for following trapezoidal trajectories.
 */
public class MoTalonFxProfilePID<Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>>
        implements MoTuner.PIDController, MoTuner.MotorFF, MoTuner.OnPopulateFinished {
    private final TalonFX motorController;
    private final TalonFXConfiguration config;

    private final Dim internalEncoderUnits;
    private final VDim internalEncoderVelocity;

    private double errorTolerance = 0.05;

    private PositionVoltage controlRequest = new PositionVoltage(0);

    @SuppressWarnings("unchecked")
    public MoTalonFxProfilePID(TalonFX controller, Dim internalEncoderUnits, TalonFXConfiguration config) {
        this.motorController = controller;
        this.config = config;

        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocity = (VDim) internalEncoderUnits.per(TalonFxEncoder.VELOCITY_BASE_UNIT);
    }

    public void applyConfigs() {
        motorController.getConfigurator().apply(config);
    }

    @Override
    public void setP(double kP) {
        config.Slot0.kP = kP;
    }

    @Override
    public void setI(double kI) {
        config.Slot0.kI = kI;
    }

    @Override
    public void setD(double kD) {
        config.Slot0.kD = kD;
    }

    @Override
    public void setS(double kS) {
        config.Slot0.kS = kS;
    }

    @Override
    public void setV(double kV) {
        config.Slot0.kV = kV;
    }

    @Override
    public void setA(double kA) {
        config.Slot0.kA = kA;
    }

    public void setTolerance(double tolerance) {
        errorTolerance = tolerance;
    }

    public double getPositionSetpoint() {
        return controlRequest.Position;
    }

    public double getVelocitySetpoint() {
        return controlRequest.Velocity;
    }

    public double getPositionMeasurement() {
        return motorController.getPosition().getValueAsDouble();
    }

    public double getVelocityMeasurement() {
        return motorController.getVelocity().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return Math.abs(getPositionSetpoint() - getPositionMeasurement()) < errorTolerance;
    }

    @Override
    public void onPopulateFinished() {
        applyConfigs();
    }

    public void setReference(Measure<Dim> position, Measure<VDim> velocity) {
        controlRequest.Position = position.in(internalEncoderUnits);
        controlRequest.Velocity = velocity.in(internalEncoderVelocity);
        this.motorController.setControl(controlRequest);
    }
}
