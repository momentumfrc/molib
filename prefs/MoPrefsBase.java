package first.molib.prefs;

import first.molib.MoUnits;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

/**
 * Base class for robot preferences.
 */
public abstract class MoPrefsBase {
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

    protected static AngularVelocityUnitPref degreesPerSecPref(String key, AngularVelocity defaultValue) {
        return new AngularVelocityUnitPref(key, Units.DegreesPerSecond, defaultValue);
    }

    protected static AngularVelocityUnitPref rpmPref(String key, AngularVelocity defaultValue) {
        return new AngularVelocityUnitPref(key, Units.RPM, defaultValue);
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

    protected static AngularAccelerationUnitPref rotationsPerSec2Pref(String key, AngularAcceleration defaultValue) {
        return new AngularAccelerationUnitPref(key, Units.RotationsPerSecondPerSecond, defaultValue);
    }

    protected static AngularAccelerationUnitPref degreesPerSec2Pref(String key, AngularAcceleration defaultValue) {
        return new AngularAccelerationUnitPref(key, Units.DegreesPerSecondPerSecond, defaultValue);
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

    protected MoPrefsBase() {
        throw new UnsupportedOperationException("MoPrefs is a static class");
    }
}
