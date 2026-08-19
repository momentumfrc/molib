package first.molib;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Units;

public class MoUnits {
    private MoUnits() {
        throw new UnsupportedOperationException("Cannot instantiate MoUnits");
    }

    public static DimensionlessUnit EncoderTicks = Units.derive(Units.Value)
            .aggregate(1)
            .named("EncoderTicks")
            .symbol("et")
            .make();

    public static PerUnit<DimensionlessUnit, DistanceUnit> EncoderTicksPerMeter = EncoderTicks.per(Units.Meter);
    public static PerUnit<DimensionlessUnit, DistanceUnit> EncoderTicksPerCentimeter =
            EncoderTicks.per(Units.Centimeter);
    public static PerUnit<DimensionlessUnit, AngleUnit> EncoderTicksPerRotation = EncoderTicks.per(Units.Rotation);
    public static PerUnit<DimensionlessUnit, AngleUnit> EncoderTicksPerRadian = EncoderTicks.per(Units.Radian);
}
