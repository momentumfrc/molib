package frc.robot.molib.prefs;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.function.BiConsumer;

public class MoPrefsUtils {

    public static <T, U> void multiSubscribe(Pref<T> pref1, Pref<U> pref2, BiConsumer<T, U> subscription) {
        multiSubscribe(pref1, pref2, subscription, false);
    }

    public static <T, U> void multiSubscribe(
            Pref<T> pref1, Pref<U> pref2, BiConsumer<T, U> subscription, boolean notifyImmediately) {
        pref1.subscribe(value1 -> subscription.accept(value1, pref2.get()), false);
        pref2.subscribe(value2 -> subscription.accept(pref1.get(), value2), false);
        if (notifyImmediately) {
            subscription.accept(pref1.get(), pref2.get());
        }
    }

    public static <T extends Unit, U extends Unit> void multiSubscribe(
            UnitPref<T> pref1, UnitPref<U> pref2, BiConsumer<Measure<T>, Measure<U>> subscription) {
        multiSubscribe(pref1, pref2, subscription, false);
    }

    public static <T extends Unit, U extends Unit> void multiSubscribe(
            UnitPref<T> pref1,
            UnitPref<U> pref2,
            BiConsumer<Measure<T>, Measure<U>> subscription,
            boolean notifyImmediately) {
        pref1.subscribe(value1 -> subscription.accept(value1, pref2.get()), false);
        pref2.subscribe(value2 -> subscription.accept(pref1.get(), value2), false);
        if (notifyImmediately) {
            subscription.accept(pref1.get(), pref2.get());
        }
    }

    private MoPrefsUtils() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [MoPrefsUtils]");
    }
}
