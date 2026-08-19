package first.molib.prefs;

import org.wpilib.units.TimeUnit;
import org.wpilib.units.measure.Time;

public class TimeUnitPref extends UnitPref<TimeUnit> {
    public TimeUnitPref(String key, TimeUnit storeUnits, Time defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Time get() {
        return (Time) super.get();
    }
}
