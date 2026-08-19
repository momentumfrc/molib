package first.molib.prefs;

import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.measure.AngularVelocity;

public class AngularVelocityUnitPref extends UnitPref<AngularVelocityUnit> {
    public AngularVelocityUnitPref(String key, AngularVelocityUnit storeUnits, AngularVelocity defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public AngularVelocity get() {
        return (AngularVelocity) super.get();
    }
}
