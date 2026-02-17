package frc.robot.molib.pid;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.molib.encoder.TalonFxEncoder;
import frc.robot.molib.motune.MoTuner;

/**
 * A MoTalonFxPID specialized for following trapezoidal trajectories.
 */
public class MoTalonFxProfilePID<Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>>
        implements MoTuner.PIDController, MoTuner.MotorFF, MoTuner.OnPopulateFinished {
    private final TalonFX motorController;
    private final Slot0Configs slotPIDConfigs = new Slot0Configs();

    private final Dim internalEncoderUnits;
    private final VDim internalEncoderVelocity;

    private double errorTolerance = 0.05;

    private PositionVoltage controlRequest = new PositionVoltage(0);

    @SuppressWarnings("unchecked")
    public MoTalonFxProfilePID(TalonFX controller, Dim internalEncoderUnits) {
        this.motorController = controller;

        // Load the motor controller's Slot 0 PID values into slotPIDConfigs
        this.motorController.getConfigurator().refresh(slotPIDConfigs);

        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocity = (VDim) internalEncoderUnits.per(TalonFxEncoder.VELOCITY_BASE_UNIT);
    }

    public Slot0Configs getConfigs() {
        return slotPIDConfigs;
    }

    public void applyConfigs() {
        motorController.getConfigurator().apply(slotPIDConfigs);
    }

    @Override
    public void setP(double kP) {
        slotPIDConfigs.kP = kP;
    }

    @Override
    public void setI(double kI) {
        slotPIDConfigs.kI = kI;
    }

    @Override
    public void setD(double kD) {
        slotPIDConfigs.kD = kD;
    }

    @Override
    public void setS(double kS) {
        slotPIDConfigs.kS = kS;
    }

    @Override
    public void setV(double kV) {
        slotPIDConfigs.kV = kV;
    }

    @Override
    public void setA(double kA) {
        slotPIDConfigs.kA = kA;
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
