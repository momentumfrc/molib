package first.molib.prefs;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringTopic;
import org.wpilib.networktables.Topic;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStationErrors;

import java.util.HashSet;
import java.util.Set;

/**
 * Singleton class holding a reference to the NetworkTable backing the MoPrefs.
 */
public class MoPrefsImpl {
    private NetworkTable backingTable;

    private static MoPrefsImpl instance;
    private StringPublisher typePublisher;
    private Set<String> activeKeys = new HashSet<>();
    private boolean cleaned = false;

    static synchronized MoPrefsImpl getInstance() {
        if (instance == null) {
            instance = new MoPrefsImpl();
        }
        return instance;
    }

    NetworkTableEntry getEntry(String key) {
        if (cleaned) {
            DriverStationErrors.reportError(
                    String.format("New pref [%s] added after cleanup - this pref will be reset at next boot", key),
                    false);
        }
        activeKeys.add(key);
        return backingTable.getEntry(key);
    }

    public static void cleanUpPrefs() {
        MoPrefsImpl instance = getInstance();
        instance.cleaned = true;
        HashSet<String> pref_keys = new HashSet<>(instance.activeKeys);

        // Shouldn't remove the special field .type
        pref_keys.add(".type");

        Set<String> table_keys = instance.backingTable.getKeys();

        System.out.println("****** Clean up MoPrefs ******");
        for (String key : table_keys) {
            if (!pref_keys.contains(key)) {
                System.out.format("Remove unused pref \"%s\"\n", key);

                Topic topic = instance.backingTable.getTopic(key);
                topic.setPersistent(false);
                topic.setRetained(false);
            }
        }
    }

    private MoPrefsImpl() {
        backingTable = NetworkTableInstance.getDefault().getTable("Preferences");

        String kSmartDashboardType = "RobotPreferences";
        typePublisher = backingTable
                .getStringTopic(".type")
                .publishEx(StringTopic.TYPE_STRING, "{\"SmartDashboard\":\"" + kSmartDashboardType + "\"}");
        typePublisher.set(kSmartDashboardType);
    }
}
