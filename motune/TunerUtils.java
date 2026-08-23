package first.robot.molib.motune;

import first.robot.molib.pid.MoSparkMaxPID;
import first.robot.molib.pid.MoTalonFxPID;
import first.robot.molib.pid.MoTalonFxProfilePID;
import java.util.Optional;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;

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
                .setpoint(talon::getSetpoint)
                .measurement(talon::getLastMeasurement)
                .parameter("tolerance", talon::setTolerance)
                .stateVariable("at_setpoint", () -> talon.atSetpoint() ? 1.0 : 0.0)
                .onPopulateFinished(talon)
                .safeBuild();
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> Optional<MoTuner> forMoTalonFxProfile(
            MoTalonFxProfilePID<Dim, VDim> talon, String name) {
        return MoTuner.builder(name)
                .pid(talon)
                .motorFF(talon)
                .parameter("tolerance", talon::setTolerance)
                .stateVariable("setpoint_pos", talon::getPositionSetpoint)
                .stateVariable("setpoint_vel", talon::getVelocitySetpoint)
                .stateVariable("measurement_pos", talon::getPositionMeasurement)
                .stateVariable("measurement_vel", talon::getVelocityMeasurement)
                .stateVariable("at_setpoint", () -> talon.atSetpoint() ? 1.0 : 0.0)
                .onPopulateFinished(talon)
                .safeBuild();
    }

    private TunerUtils() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [TunerUtils]");
    }
}
