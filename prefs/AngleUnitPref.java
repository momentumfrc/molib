package first.robot.molib.prefs;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.measure.Angle;

public class AngleUnitPref extends UnitPref<AngleUnit> {
    public AngleUnitPref(String key, AngleUnit storeUnits, Angle defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Angle get() {
        return (Angle) super.get();
    }
}
