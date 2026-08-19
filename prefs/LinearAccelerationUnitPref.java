package first.molib.prefs;

import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.measure.LinearAcceleration;

public class LinearAccelerationUnitPref extends UnitPref<LinearAccelerationUnit> {
    public LinearAccelerationUnitPref(String key, LinearAccelerationUnit storeUnits, LinearAcceleration defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public LinearAcceleration get() {
        return (LinearAcceleration) super.get();
    }
}
