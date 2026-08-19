package first.molib.prefs;

import java.util.function.BiConsumer;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class MoPrefsUtils {

    @FunctionalInterface
    public interface TriConsumer<T, U, V> {
        public void accept(T t, U u, V v);
    }

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

    public static <T, U, V> void multiSubscribe(
            Pref<T> pref1, Pref<U> pref2, Pref<V> pref3, TriConsumer<T, U, V> subscription) {
        multiSubscribe(pref1, pref2, pref3, subscription, false);
    }

    public static <T, U, V> void multiSubscribe(
            Pref<T> pref1, Pref<U> pref2, Pref<V> pref3, TriConsumer<T, U, V> subscription, boolean notifyImmediately) {
        pref1.subscribe(value1 -> subscription.accept(value1, pref2.get(), pref3.get()), false);
        pref2.subscribe(value2 -> subscription.accept(pref1.get(), value2, pref3.get()), false);
        pref3.subscribe(value3 -> subscription.accept(pref1.get(), pref2.get(), value3), false);
        if (notifyImmediately) {
            subscription.accept(pref1.get(), pref2.get(), pref3.get());
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

    public static <T extends Unit, U extends Unit, V extends Unit> void multiSubscribe(
            UnitPref<T> pref1,
            UnitPref<U> pref2,
            UnitPref<V> pref3,
            TriConsumer<Measure<T>, Measure<U>, Measure<V>> subscription) {
        multiSubscribe(pref1, pref2, pref3, subscription, false);
    }

    public static <T extends Unit, U extends Unit, V extends Unit> void multiSubscribe(
            UnitPref<T> pref1,
            UnitPref<U> pref2,
            UnitPref<V> pref3,
            TriConsumer<Measure<T>, Measure<U>, Measure<V>> subscription,
            boolean notifyImmediately) {
        pref1.subscribe(value1 -> subscription.accept(value1, pref2.get(), pref3.get()), false);
        pref2.subscribe(value2 -> subscription.accept(pref1.get(), value2, pref3.get()), false);
        pref3.subscribe(value3 -> subscription.accept(pref1.get(), pref2.get(), value3), false);
        if (notifyImmediately) {
            subscription.accept(pref1.get(), pref2.get(), pref3.get());
        }
    }

    private MoPrefsUtils() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [MoPrefsUtils]");
    }
}
