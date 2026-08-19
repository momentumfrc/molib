package first.molib.prefs;

import java.util.function.Consumer;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class UnitPref<U extends Unit> {
    private final Pref<Double> basePref;
    private final U storeUnits;

    private Measure<U> currValue;

    public UnitPref(String key, U storeUnits, Measure<U> defaultValue) {
        String symbol = storeUnits.symbol().replaceAll("/", "_");

        currValue = defaultValue;

        this.basePref = new Pref<>(
                String.format("%s (%s)", key, symbol),
                defaultValue.in(storeUnits),
                NetworkTableValue::getDouble,
                NetworkTableEntry::setDouble);

        this.storeUnits = storeUnits;
    }

    public Measure<U> get() {
        return replace(basePref.get(), storeUnits);
    }

    public Measure<U> replace(double updated, U storeUnits) {
        currValue = (Measure<U>) storeUnits.of(basePref.get());
        return currValue;
    }

    public void set(Measure<U> value) {
        basePref.set(value.in(storeUnits));
    }

    public void subscribe(Consumer<Measure<U>> consumer) {
        subscribe(consumer, false);
    }

    public void subscribe(Consumer<Measure<U>> consumer, boolean notifyImmediately) {
        basePref.subscribe((value) -> consumer.accept(replace(value, storeUnits)), notifyImmediately);
    }

    public String getKey() {
        return basePref.getKey();
    }
}
