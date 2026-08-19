package first.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import first.robot.molib.MoSparkConfigurator;
import first.robot.molib.encoder.MoEncoder;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

public class MoSparkMaxArmProfilePID extends MoSparkMaxPID<AngleUnit, AngularVelocityUnit> {
    private static final double REPATH_ERROR_DEGS = 15;
    private static final double LOOP_PERIOD = 0.02;

    // TODO run feedforward and profile calculations on-spark
    private TrapezoidProfile profile = null;
    private TrapezoidProfile.Constraints profileConstraints = new TrapezoidProfile.Constraints(0, 0);
    private TrapezoidProfile.State setpointState = null;

    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

    private Angle horizontalOffset = Units.Rotations.of(0);

    private double lastFF = 0;

    public MoSparkMaxArmProfilePID(
            SparkBase controller,
            ClosedLoopSlot pidSlot,
            MoEncoder<AngleUnit, AngularVelocityUnit> internalEncoder,
            MoSparkConfigurator configurator) {
        super(Type.POSITION, controller, pidSlot, internalEncoder, configurator);
    }

    @Override
    public void setS(double kS) {
        super.setS(0);
        feedforward.setKs(kS);
    }

    @Override
    public void setV(double kV) {
        super.setV(0);
        feedforward.setKv(kV);
    }

    @Override
    public void setA(double kA) {
        super.setA(0);
        feedforward.setKa(kA);
    }

    public void setG(double kG) {
        feedforward.setKg(kG);
    }

    /**
     * This value is summed with the reported encoder angle to get the angle of the arm from the horizon.
     */
    public void setHorizontalOffset(double offset) {
        horizontalOffset = internalEncoder.getInternalEncoderUnits().of(offset);
    }

    public void setMaxVelocity(AngularVelocity velocity) {
        profile = null;
        profileConstraints = new TrapezoidProfile.Constraints(
                velocity.in(Units.RadiansPerSecond), profileConstraints.maxAcceleration);
    }

    public void setMaxAcceleration(AngularAcceleration acceleration) {
        profile = null;
        profileConstraints = new TrapezoidProfile.Constraints(
                profileConstraints.maxVelocity, acceleration.in(Units.RadiansPerSecondPerSecond));
    }

    public double getLastFF() {
        return lastFF;
    }

    public double calculateFF(AngularVelocity velocityReference) {
        double currPosFromHorizon =
                internalEncoder.getPosition().in(Units.Radians) + horizontalOffset.in(Units.Radians);
        return feedforward.calculate(currPosFromHorizon, velocityReference.in(Units.RadiansPerSecond));
    }

    @Override
    public void setPositionReference(Measure<AngleUnit> position) {
        if (profile == null) {
            profile = new TrapezoidProfile(profileConstraints);
        }

        if (setpointState == null
        /* || Math.abs(internalEncoder.getPosition().in(Units.Degrees) - setpointState.position)
        > REPATH_ERROR_DEGS */ ) {
            setpointState = new TrapezoidProfile.State(
                    internalEncoder.getPosition().in(Units.Radians),
                    internalEncoder.getVelocity().in(Units.RadiansPerSecond));
            System.out.println("REPLAN");
        }

        var goalState = new TrapezoidProfile.State(position.in(Units.Radians), 0);

        setpointState = profile.calculate(LOOP_PERIOD, setpointState, goalState);

        double ff = calculateFF(Units.RadiansPerSecond.of(setpointState.velocity));

        double setpointNative = Units.Radians.of(setpointState.position).in(internalEncoder.getInternalEncoderUnits());
        pidController.setSetpoint(setpointNative, type.innerType, pidSlot, ff);

        lastSetpoint = setpointNative;
        lastFF = ff;
    }

    public void setUnprofiledPositionReference(Measure<AngleUnit> position) {
        double ff = calculateFF(Units.RadiansPerSecond.zero());
        double setpoint = position.in(internalEncoder.getInternalEncoderUnits());
        pidController.setSetpoint(setpoint, type.innerType, pidSlot, ff);
        lastSetpoint = setpoint;
        lastFF = ff;
    }

    @Override
    public void setVelocityReference(Measure<AngularVelocityUnit> velocity) {
        throw new UnsupportedOperationException("MoSparkMaxArmProfilePID only supports position references");
    }
}
