package first.robot.molib.prefs;

import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.measure.LinearVelocity;

public class LinearVelocityUnitPref extends UnitPref<LinearVelocityUnit> {
    public LinearVelocityUnitPref(String key, LinearVelocityUnit storeUnits, LinearVelocity defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public LinearVelocity get() {
        return (LinearVelocity) super.get();
    }
}
