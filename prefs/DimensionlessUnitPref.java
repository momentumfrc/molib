package first.molib.prefs;

import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.measure.Dimensionless;

public class DimensionlessUnitPref extends UnitPref<DimensionlessUnit> {
    public DimensionlessUnitPref(String key, DimensionlessUnit storeUnits, Dimensionless defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Dimensionless get() {
        return (Dimensionless) super.get();
    }
}
