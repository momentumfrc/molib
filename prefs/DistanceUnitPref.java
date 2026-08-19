package first.robot.molib.prefs;

import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

public class DistanceUnitPref extends UnitPref<DistanceUnit> {
    public DistanceUnitPref(String key, DistanceUnit storeUnits, Distance defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Distance get() {
        return (Distance) super.get();
    }
}
