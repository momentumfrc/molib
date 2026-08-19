// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.molib.pid;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import first.molib.encoder.TalonFxEncoder;
import first.molib.motune.MoTuner;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;

public class MoTalonFxPID<Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>>
        implements MoTuner.PIDController, MoTuner.MotorFF, MoTuner.OnPopulateFinished {
    private final Type type;
    private final TalonFX motorController;
    private final Slot0Configs slotPIDConfigs = new Slot0Configs();
    private double lastReference;

    private Dim internalEncoderUnits;
    protected final VDim internalEncoderVelocity;

    private double errorTolerance = 0.05;

    @SuppressWarnings("unchecked")
    public MoTalonFxPID(Type type, TalonFX controller, Dim internalEncoderUnits) {
        this.type = type;
        this.motorController = controller;

        // Load the motor controller's Slot 0 PID values into slotPIDConfigs
        this.motorController.getConfigurator().refresh(slotPIDConfigs);

        this.internalEncoderUnits = internalEncoderUnits;
        this.internalEncoderVelocity = (VDim) internalEncoderUnits.per(TalonFxEncoder.VELOCITY_BASE_UNIT);
    }

    public Type getType() {
        return type;
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

    @Override
    public void onPopulateFinished() {
        applyConfigs();
    }

    public void setIZone(double iZone) {
        if (iZone != 0) {
            DriverStationErrors.reportError(
                    "Cannot set iZone on TalonFx, it is not supported by the Phoenix API", true);
        }
    }

    public double getLastOutput() {
        return this.motorController.getBridgeOutput().getValueAsDouble();
    }

    public double getSetpoint() {
        return this.lastReference;
    }

    public double getLastMeasurement() {
        switch (this.type) {
            case POSITION:
            case SMARTMOTION:
                return this.motorController.getPosition().getValueAsDouble();
            case VELOCITY:
                return this.motorController.getVelocity().getValueAsDouble();
        }

        return 0;
    }

    public boolean atSetpoint() {
        return Math.abs(getSetpoint() - getLastMeasurement()) < errorTolerance;
    }

    /**
     * Set the reference of the PID controller. The units of value depend on the current type of the controller.
     * For position controllers (Type.POSITION or Type.SMARTMOTION), value is measured in internalEncoderUnits.
     * For velocity controllers (Type.VELOCITY or Type.SMARTVELOCITY), value is measured in internalEncoderUnits per second.
     * <p>
     * @deprecated Use {@link #setPositionReference(Measure)} or {@link #setVelocityReference(Measure)}
     */
    @Deprecated(forRemoval = false)
    public void setReference(double value) {
        this.motorController.setControl(this.type.holder.apply(value));
        this.lastReference = value;
    }

    public void setPositionReference(Measure<Dim> position) {
        if (this.type != Type.POSITION && this.type != Type.SMARTMOTION) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set position on PID controller of type %s", this.type.name()));
        }
        double value = position.in(internalEncoderUnits);
        this.motorController.setControl(this.type.holder.apply(value));
        lastReference = value;
    }

    public void setVelocityReference(Measure<VDim> velocity) {
        if (this.type != Type.VELOCITY) {
            throw new UnsupportedOperationException(
                    String.format("Cannot set velocity on PID controller of type %s", this.type.name()));
        }

        double value = velocity.in(internalEncoderVelocity);
        this.motorController.setControl(this.type.holder.apply(value));
        lastReference = value;
    }

    private static record ControlRequestHolder<T extends ControlRequest>(
            T controlRequest, ControlRequestHolder.ApplyControlRequest<T> applyControlRequest) {
        @FunctionalInterface
        public static interface ApplyControlRequest<T extends ControlRequest> {
            T apply(T request, double value);
        }

        public T apply(double value) {
            return this.applyControlRequest.apply(controlRequest, value);
        }
    }

    public enum Type {
        POSITION(new ControlRequestHolder<PositionVoltage>(new PositionVoltage(0), PositionVoltage::withPosition)),
        SMARTMOTION(new ControlRequestHolder<MotionMagicVoltage>(
                new MotionMagicVoltage(0), MotionMagicVoltage::withPosition)),
        VELOCITY(new ControlRequestHolder<VelocityVoltage>(new VelocityVoltage(0), VelocityVoltage::withVelocity));

        private final ControlRequestHolder<?> holder;

        private Type(ControlRequestHolder<?> holder) {
            this.holder = holder;
        }
    }
}
