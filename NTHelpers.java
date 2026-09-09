package first.robot.molib;

import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableRegistry;

public class NTHelpers {

    public static NetworkTable getTable(String name) {
        return NetworkTableInstance.getDefault().getTable(name);
    }

    /**
     * Return an entry representing a boolean value on the network tables, with an initial defaultValue set
     * on robot startup.
     * <p>
     * The intention behind this helper is to provide a shortcut for getting an entry and publishing an initial value.
     * This should for values that will be set on the dashboard and propagated to the robot.
     */
    public static BooleanEntry getBooleanEntry(NetworkTable table, String name, boolean defaultValue) {
        var entry = table.getBooleanTopic(name).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }

    /**
     * Return an entry representing a double value on the network tables, with an initial defaultValue set
     * on robot startup.
     * <p>
     * The intention behind this helper is to provide a shortcut for getting an entry and publishing an initial value.
     * This should for values that will be set on the dashboard and propagated to the robot.
     */
    public static DoubleEntry getDoubleEntry(NetworkTable table, String name, double defaultValue) {
        var entry = table.getDoubleTopic(name).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }

    public static <T extends Enum<?>> Selectable<T> enumToChooser(Class<T> toConvert) {
        return enumToChooser(toConvert, toConvert.getEnumConstants()[0]);
    }

    public static <T extends Enum<?>> Selectable<T> enumToChooser(Class<T> toConvert, T defaultValue) {
        var chooser = new Selectable<T>();
        chooser.addDefault(defaultValue.name(), defaultValue);
        for (T entry : toConvert.getEnumConstants()) {
            if (entry != defaultValue) {
                chooser.addDefault(entry.name(), entry);
            }
        }
        return chooser;
    }

    public static void publishSendable(NetworkTable table, String key, ComplexTunable data) {
        TunableRegistry.publish(table.getPath() + key, data);
    }

    private NTHelpers() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [NTHelpers]");
    }
}
