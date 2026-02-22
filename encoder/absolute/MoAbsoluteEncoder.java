package frc.robot.molib.encoder.absolute;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.molib.encoder.MoRotationEncoder;

public class MoAbsoluteEncoder {
    private final MoRotationEncoder encoder;
    private Angle encoderZero;
    private Angle encoderWraparoundRange;

    private MutAngle measure;

    public MoAbsoluteEncoder(MoRotationEncoder encoder) {
        this.encoder = encoder;

        this.encoderZero = encoder.getInternalEncoderUnits().zero();
        this.encoderWraparoundRange = Units.Rotations.of(0.2);
        measure = Units.Rotations.mutable(0);
    }

    public MoRotationEncoder getMoEncoder() {
        return encoder;
    }

    public void setEncoderZero(Angle zero) {
        this.encoderZero = zero;
    }

    public void setEncoderWraparoundRange(Angle encoderWraparoundRange) {
        this.encoderWraparoundRange = encoderWraparoundRange;
    }

    public Angle getPosition() {
        double pos = encoder.getPosition().in(Units.Rotations);
        pos = (pos + 1 - encoderZero.in(Units.Rotations)) % 1;

        double wrapRange = encoderWraparoundRange.in(Units.Rotations);
        if (wrapRange > 0 && pos > (1 - wrapRange)) {
            pos -= 1;
        }
        return measure.mut_replace(pos, Units.Rotations);
    }

    public static MoAbsoluteEncoder forDio(int dioPort) {
        return new MoAbsoluteEncoder(MoRotationEncoder.forDioAbsolute(dioPort));
    }

    public static MoAbsoluteEncoder forSpark(SparkBase spark, AngleUnit internalEncoderUnits) {
        return new MoAbsoluteEncoder(MoRotationEncoder.forSparkAbsolute(spark, internalEncoderUnits));
    }
}
