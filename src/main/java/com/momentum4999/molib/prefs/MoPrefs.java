package com.momentum4999.molib.prefs;

import com.momentum4999.molib.MoUnits;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

/**
 * Base class for robot preferences.
 */
public abstract class MoPrefs {
    protected static Pref<Boolean> booleanPref(String key, boolean defaultValue) {
        return new Pref<>(key, defaultValue, NetworkTableValue::getBoolean, NetworkTableEntry::setBoolean);
    }

    protected static Pref<Double> unitlessDoublePref(String key, double defaultValue) {
        return new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }

    protected static AngleUnitPref rotationsPref(String key, Angle defaultValue) {
        return new AngleUnitPref(key, Units.Rotations, defaultValue);
    }

    protected static AngleUnitPref degreesPref(String key, Angle defaultValue) {
        return new AngleUnitPref(key, Units.Degrees, defaultValue);
    }

    protected static DistanceUnitPref metersPref(String key, Distance defaultValue) {
        return new DistanceUnitPref(key, Units.Meters, defaultValue);
    }

    protected static DistanceUnitPref inchesPref(String key, Distance defaultValue) {
        return new DistanceUnitPref(key, Units.Inches, defaultValue);
    }

    protected static DistanceUnitPref centimetersPref(String key, Distance defaultValue) {
        return new DistanceUnitPref(key, Units.Centimeters, defaultValue);
    }

    protected static LinearVelocityUnitPref metersPerSecPref(String key, LinearVelocity defaultValue) {
        return new LinearVelocityUnitPref(key, Units.MetersPerSecond, defaultValue);
    }

    protected static AngularVelocityUnitPref rotationsPerSecPref(String key, AngularVelocity defaultValue) {
        return new AngularVelocityUnitPref(key, Units.RotationsPerSecond, defaultValue);
    }

    protected static TimeUnitPref secondsPref(String key, Time defaultValue) {
        return new TimeUnitPref(key, Units.Seconds, defaultValue);
    }

    protected static DimensionlessUnitPref percentPref(String key, Dimensionless defaultValue) {
        return new DimensionlessUnitPref(key, Units.Percent, defaultValue);
    }

    protected static LinearAccelerationUnitPref metersPerSecPerSecPref(String key, LinearAcceleration defaultValue) {
        return new LinearAccelerationUnitPref(key, Units.MetersPerSecondPerSecond, defaultValue);
    }

    protected static AngularAccelerationUnitPref rotationsPerSecPerSecPref(
            String key, AngularAcceleration defaultValue) {
        return new AngularAccelerationUnitPref(key, Units.RotationsPerSecondPerSecond, defaultValue);
    }

    protected static UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> encoderTicksPerCentimeterPref(
            String key, Measure<PerUnit<DimensionlessUnit, DistanceUnit>> defaultValue) {
        return new UnitPref<>(key, MoUnits.EncoderTicksPerCentimeter, defaultValue);
    }

    protected static UnitPref<PerUnit<DimensionlessUnit, DistanceUnit>> encoderTicksPerMeterPref(
            String key, Measure<PerUnit<DimensionlessUnit, DistanceUnit>> defaultValue) {
        return new UnitPref<>(key, MoUnits.EncoderTicksPerMeter, defaultValue);
    }

    protected static UnitPref<PerUnit<DimensionlessUnit, AngleUnit>> encoderTicksPerRotationPref(
            String key, Measure<PerUnit<DimensionlessUnit, AngleUnit>> defaultValue) {
        return new UnitPref<>(key, MoUnits.EncoderTicksPerRotation, defaultValue);
    }

    protected static UnitPref<CurrentUnit> ampsPref(String key, Measure<CurrentUnit> defaultValue) {
        return new UnitPref<>(key, Units.Amps, defaultValue);
    }

    protected static UnitPref<VoltageUnit> voltsPref(String key, Measure<VoltageUnit> defaultValue) {
        return new UnitPref<>(key, Units.Volts, defaultValue);
    }

    protected MoPrefs() {
        throw new UnsupportedOperationException("MoPrefs is a static class");
    }
}
