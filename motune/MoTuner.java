package frc.robot.molib.motune;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Utility class for storing controller parameters and state values in NetworkTables.
 * <p>
 * Parameters are persistent and on-the-fly updates are propagated to the associated underlying controllers. State
 * values are periodically polled and the associated NetworkTable entries are kept up to date.
 */
public class MoTuner implements NetworkTable.TableEventListener {
    public static final String TUNER_TABLE = "momentum-tuners";
    private static final double DEFAULT_VALUE = 0;
    private static final List<MoTuner> instances = new ArrayList<>();

    private static void recordInstance(MoTuner tuner) {
        instances.add(tuner);
    }

    public static Builder builder(String name) {
        return new Builder(name);
    }

    /**
     * Update the NetworkTables topic with the latest value for every configured state variable.
     */
    public static void pollAllStateValues() {
        for (MoTuner instance : instances) {
            instance.pollStateVariables();
        }
    }

    /**
     * A value that is kept constant during operation, but may need to be tuned. <i>(kP, kI, iZone, etc.)</i>
     */
    public record PIDParameter(String name, DoubleConsumer setter) {}

    /**
     * A value that changes during the operation of the controller. <i>(position, velocity, setpoint, etc.)</i>
     */
    public record PIDStateVariable(String name, DoubleSupplier getter) {}

    public interface PIDController {
        void setP(double kP);

        void setI(double kI);

        void setD(double kD);
    }

    public interface MotorFF {
        void setS(double kS);

        void setV(double kV);

        void setA(double kA);
    }

    /**
     * Called after populating values from networktables into PIDParameters.
     */
    @FunctionalInterface
    public interface OnPopulateFinished {
        void onPopulateFinished();

        static OnPopulateFinished empty() {
            return () -> {};
        }
    }

    public static class Builder {
        private final String name;
        private Map<String, PIDParameter> parameters = new HashMap<>();
        private Map<String, PIDStateVariable> stateVariables = new HashMap<>();
        private OnPopulateFinished populateHook = null;

        Builder(String name) {
            this.name = name;
        }

        public Builder parameter(PIDParameter param) {
            if (parameters.containsKey(param.name())) {
                throw new IllegalArgumentException(
                        "Duplicate parameter [" + param.name() + "] for controller [" + name + "]");
            }

            parameters.put(param.name(), param);
            return this;
        }

        public Builder stateVariable(PIDStateVariable stateVar) {
            if (stateVariables.containsKey(stateVar.name())) {
                throw new IllegalArgumentException(
                        "Duplicate state variable [" + stateVar.name() + "] for controller + [" + name + "]");
            }

            stateVariables.put(stateVar.name(), stateVar);
            return this;
        }

        public Builder parameter(String name, DoubleConsumer setter) {
            return parameter(new PIDParameter(name, setter));
        }

        public Builder stateVariable(String name, DoubleSupplier getter) {
            return stateVariable(new PIDStateVariable(name, getter));
        }

        public Builder p(DoubleConsumer setter) {
            return parameter("kP", setter);
        }

        public Builder i(DoubleConsumer setter) {
            return parameter("kI", setter);
        }

        public Builder d(DoubleConsumer setter) {
            return parameter("kD", setter);
        }

        public Builder pid(PIDController pid) {
            p(pid::setP);
            i(pid::setI);
            d(pid::setD);
            return this;
        }

        public Builder ff_s(DoubleConsumer setter) {
            return parameter("ff_kS", setter);
        }

        public Builder ff_v(DoubleConsumer setter) {
            return parameter("ff_kV", setter);
        }

        public Builder ff_a(DoubleConsumer setter) {
            return parameter("ff_kA", setter);
        }

        public Builder motorFF(MotorFF ff) {
            ff_s(ff::setS);
            ff_v(ff::setV);
            ff_a(ff::setA);
            return this;
        }

        public Builder iZone(DoubleConsumer setter) {
            return parameter("i_zone", setter);
        }

        public Builder setpoint(DoubleSupplier getter) {
            return stateVariable("setpoint", getter);
        }

        public Builder measurement(DoubleSupplier getter) {
            return stateVariable("measurement", getter);
        }

        public Builder onPopulateFinished(OnPopulateFinished populateHook) {
            if (this.populateHook != null) {
                throw new IllegalArgumentException("Duplicate OnPopulateFinished hook for [" + name + "]");
            }
            this.populateHook = populateHook;
            return this;
        }

        public MoTuner build() {
            var hook = populateHook != null ? populateHook : OnPopulateFinished.empty();
            var tuner = new MoTuner(name, parameters, stateVariables, hook);
            recordInstance(tuner);
            return tuner;
        }

        public Optional<MoTuner> safeBuild() {
            try {
                return Optional.of(build());
            } catch (Exception e) {
                DriverStation.reportError("Failed to build tuner for controller [" + name + "]", e.getStackTrace());
                return Optional.empty();
            }
        }
    }

    private final String name;
    private final Map<String, PIDParameter> parameters;
    private final Map<String, PIDStateVariable> stateVariables;
    private final OnPopulateFinished populateHook;

    private final NetworkTable table;
    private final int listenerHandle;

    private final Map<PIDParameter, NetworkTableEntry> parameterEntries;
    private Map<PIDStateVariable, DoublePublisher> stateVariablePublishers = null;

    private MoTuner(
            String name,
            Map<String, PIDParameter> parameters,
            Map<String, PIDStateVariable> stateVariables,
            OnPopulateFinished populateHook) {
        this.name = name;

        this.parameters = Collections.unmodifiableMap(parameters);
        this.stateVariables = Collections.unmodifiableMap(stateVariables);
        this.populateHook = populateHook;

        table = NetworkTableInstance.getDefault().getTable(TUNER_TABLE).getSubTable(name);
        listenerHandle = table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), this);

        var parameterEntries = new HashMap<PIDParameter, NetworkTableEntry>();
        for (var mapEntry : parameters.entrySet()) {
            var entry = table.getEntry(mapEntry.getKey());
            entry.setDefaultDouble(DEFAULT_VALUE);
            entry.setPersistent();
            parameterEntries.put(mapEntry.getValue(), entry);
        }
        this.parameterEntries = Collections.unmodifiableMap(parameterEntries);

        refreshParameters();
    }

    /**
     * Retrieve the value of every configured parameter.
     */
    public void refreshParameters() {
        for (var param : parameters.values()) {
            double value = parameterEntries.get(param).getDouble(DEFAULT_VALUE);
            param.setter().accept(value);
        }
        this.populateHook.onPopulateFinished();
    }

    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        assert event.is(Kind.kValueAll);

        double value = event.valueData.value.getDouble();

        var param = parameters.get(key);
        if (param == null) {
            return;
        }

        param.setter().accept(value);
        this.populateHook.onPopulateFinished();
    }

    private Map<PIDStateVariable, DoublePublisher> getStateVariablePublishers() {
        if (this.stateVariablePublishers == null) {
            var publishers = new HashMap<PIDStateVariable, DoublePublisher>();
            for (var stateVar : stateVariables.values()) {
                var publisher = table.getDoubleTopic(stateVar.name()).publish();
                publishers.put(stateVar, publisher);
            }
            this.stateVariablePublishers = Collections.unmodifiableMap(publishers);
        }
        return this.stateVariablePublishers;
    }

    private void pollStateVariables() {
        for (var entry : getStateVariablePublishers().entrySet()) {
            entry.getValue().set(entry.getKey().getter().getAsDouble());
        }
    }
}
