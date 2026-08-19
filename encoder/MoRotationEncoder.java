package first.robot.molib.encoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import first.robot.molib.MoSparkConfigurator;
import java.util.function.Consumer;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class MoRotationEncoder extends MoEncoder<AngleUnit, AngularVelocityUnit> {
    MoRotationEncoder(Encoder encoder, AngleUnit internalEncoderUnits) {
        super(encoder, internalEncoderUnits);
    }

    @Override
    public Angle getPosition() {
        return (Angle) super.getPosition();
    }

    @Override
    public AngularVelocity getVelocity() {
        return (AngularVelocity) super.getVelocity();
    }

    public static MoRotationEncoder forSparkRelative(SparkBase spark, AngleUnit internalEncoderUnits) {
        return forSparkRelative(spark.getEncoder(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static MoRotationEncoder forSparkRelative(
            RelativeEncoder encoder, AngleUnit internalEncoderUnits, Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoRotationEncoder(new RevRelativeEncoder(encoder, configurator), internalEncoderUnits);
    }

    public static MoRotationEncoder forSparkAbsolute(SparkBase spark, AngleUnit internalEncoderUnits) {
        return forSparkAbsolute(
                spark.getAbsoluteEncoder(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static MoRotationEncoder forSparkAbsolute(
            AbsoluteEncoder encoder, AngleUnit internalEncoderUnits, Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoRotationEncoder(new RevAbsoluteEncoder(encoder, configurator), internalEncoderUnits);
    }

    public static MoRotationEncoder forSparkAnalog(SparkBase spark, AngleUnit internalEncoderUnits) {
        return forSparkAnalog(spark.getAnalog(), internalEncoderUnits, MoSparkConfigurator.forSparkBase(spark));
    }

    public static MoRotationEncoder forSparkAnalog(
            SparkAnalogSensor sensor,
            AngleUnit internalEncoderUnits,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        return new MoRotationEncoder(new RevAnalogSensorEncoder(sensor, configurator), internalEncoderUnits);
    }

    public static MoRotationEncoder forTalonFx(
            TalonFX talon, AngleUnit internalEncoderUnits, TalonFXConfiguration config) {
        return new MoRotationEncoder(new TalonFxEncoder(talon, config), internalEncoderUnits);
    }

    public static MoRotationEncoder forDioAbsolute(int dioPort) {
        return new MoRotationEncoder(new DIOAbsEncoder(dioPort), Units.Rotations);
    }
}
