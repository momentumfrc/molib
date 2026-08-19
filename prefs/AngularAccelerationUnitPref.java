package first.molib.prefs;

import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.measure.AngularAcceleration;

public class AngularAccelerationUnitPref extends UnitPref<AngularAccelerationUnit> {
    public AngularAccelerationUnitPref(
            String key, AngularAccelerationUnit storeUnits, AngularAcceleration defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public AngularAcceleration get() {
        return (AngularAcceleration) super.get();
    }
}
