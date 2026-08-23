package first.robot.molib;

import first.robot.molib.encoder.MoRotationEncoder;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Time;

public class Utils {
    private static final double ENCODER_ZERO_ZONE = 0.2;

    public static void setupRelativeEncoder(
            MoRotationEncoder encoder,
            Angle absPos,
            Measure<AngleUnit> absZero,
            Measure<PerUnit<DimensionlessUnit, AngleUnit>> ratio) {
        encoder.setConversionFactor(ratio);

        double pos = absPos.in(Units.Rotations);
        pos = (pos + 1 - absZero.in(Units.Rotations)) % 1;
        if (pos > (1 - ENCODER_ZERO_ZONE)) {
            pos -= 1;
        }
        encoder.setPosition(Units.Rotations.of(pos));
    }

    public static double curve(double val, double curve) {
        if (curve == 0) {
            return val;
        }

        return Math.signum(val) * Math.pow(Math.abs(val), curve);
    }

    /**
     * Decorates the provided command with a timeout. If the specified timeout is exceeded before the command
     * finishes normally, the command will be interrupted and un-scheduled.
     * <p>
     * The timeout is retrieved when the command is scheduled.
     */
    public static Command withTimeoutPref(Command command, Supplier<Time> timeoutSupplier) {
        return command.raceWith(Commands.defer(() -> Commands.waitTime(timeoutSupplier.get()), Set.of()));
    }

    private Utils() {
        throw new UnsupportedOperationException("MoUtils is a static class");
    }
}
