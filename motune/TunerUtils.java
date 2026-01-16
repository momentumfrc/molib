package frc.robot.molib.motune;

import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.molib.pid.MoSparkMaxPID;
import frc.robot.molib.pid.MoTalonFxPID;
import java.util.Optional;

public class TunerUtils {

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> Optional<MoTuner> forMoSparkMax(
            MoSparkMaxPID<Dim, VDim> sparkMax, String name) {
        return MoTuner.builder(name)
                .pid(sparkMax)
                .motorFF(sparkMax)
                .iZone(sparkMax::setIZone)
                .setpoint(sparkMax::getSetpoint)
                .measurement(sparkMax::getLastMeasurement)
                .onPopulateFinished(sparkMax)
                .safeBuild();
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> Optional<MoTuner> forMoTalonFx(
            MoTalonFxPID<Dim, VDim> talon, String name) {
        return MoTuner.builder(name)
                .pid(talon)
                .motorFF(talon)
                .iZone(talon::setIZone)
                .setpoint(talon::getSetpoint)
                .measurement(talon::getLastMeasurement)
                .onPopulateFinished(talon)
                .safeBuild();
    }

    private TunerUtils() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [TunerUtils]");
    }
}
