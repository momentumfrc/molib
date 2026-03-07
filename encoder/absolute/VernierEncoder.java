package frc.robot.molib.encoder.absolute;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class VernierEncoder {
    private final MoAbsoluteEncoder encoder1;
    private final MoAbsoluteEncoder encoder2;
    private final GearRatios ratios;
    private final Angle range;

    private final MutAngle measurement;
    private Angle encoderWraparoundRange;

    public record GearRatios(int mainGear, int gear1, int gear2) {
        private static int gcd(int a, int b) {
            if (b == 0) {
                return a;
            }
            return gcd(b, a % b);
        }

        private static int lcm(int a, int b) {
            return Math.abs(a * b) / gcd(a, b);
        }

        public Angle getRange() {
            return Units.Rotations.of(lcm(gear1, gear2) / ((double) mainGear));
        }

        public double ratio1() {
            return gear1 / ((double) mainGear);
        }

        public double ratio2() {
            return gear2 / ((double) mainGear);
        }
    }

    public VernierEncoder(MoAbsoluteEncoder encoder1, MoAbsoluteEncoder encoder2, GearRatios ratios) {
        this.encoder1 = encoder1;
        this.encoder2 = encoder2;

        encoder1.setEncoderWraparoundRange(Units.Rotations.zero());
        encoder2.setEncoderWraparoundRange(Units.Rotations.zero());

        this.ratios = ratios;
        this.range = ratios.getRange();

        measurement = Units.Rotations.mutable(0);

        this.encoderWraparoundRange = Units.Rotations.of(0.2);
    }

    public void setEncoderWraparoundRange(Angle encoderWraparoundRange) {
        this.encoderWraparoundRange = encoderWraparoundRange;
    }

    private double applyWraparoundRange(double value) {
        double wrapRange = encoderWraparoundRange.in(Units.Rotations);
        if (wrapRange == 0) {
            return value;
        }

        double range = this.range.in(Units.Rotations);

        if (value > (range - wrapRange)) {
            return value - range;
        }
        return value;
    }

    public Optional<Angle> getPosition() {
        double r1 = ratios.ratio1();
        double r2 = ratios.ratio2();
        double range = this.range.in(Units.Rotations);

        double measure1 = encoder1.getPosition().in(Units.Rotations);
        double measure2 = encoder2.getPosition().in(Units.Rotations);

        int revolutions1 = 0;
        int revolutions2 = 0;
        while (revolutions1 * r1 < range && revolutions2 * r2 < range) {
            double guess1 = r1 * (revolutions1 + measure1);
            double guess2 = r2 * (revolutions2 + measure2);
            if (Math.abs(guess1 - guess2) < 5e-3) {
                return Optional.of(measurement.mut_replace(applyWraparoundRange(guess1), Units.Rotations));
            }
            if (guess1 < guess2) {
                revolutions1 += 1;
            } else {
                revolutions2 += 1;
            }
        }

        DriverStation.reportError("failed to solve vernier encoder", false);
        return Optional.empty();
    }
}
