package team5427.frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
    
    @AutoLog
    public static class TurretIOInputs{
        public Rotation2d pivotMotorAngle = new Rotation2d();

        public AngularVelocity pivotMotorAngularVelocity = RotationsPerSecond.of(0.0);
        public AngularAcceleration pivotMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);

        public LinearVelocity pivotMotorLinearVelocity = MetersPerSecond.of(0.0);
        public LinearAcceleration pivotMotorLinearAcceleration = MetersPerSecondPerSecond.of(0.0);

        public Temperature pivotMotorTemperature = Celsius.of(0.0);
        public Current pivotMotorCurrent = Amps.of(0.0);
        public Voltage pivotMotorVoltage = Volts.of(0.0);

        public boolean pivotMotorIsConnected = false;

        public Rotation2d rollerMotorAngle = new Rotation2d();

        public Temperature rollerMotorTemperature = Celsius.of(0.0);
        public Current rollerMotorCurrent = Amps.of(0.0);
        public Voltage rollerMotorVoltage = Volts.of(0.0);

        public boolean rollerMotorIsConnected = false;
    }

    public void updateInputs(TurretIOInputs inputs);

    public void setpivotRotation(Rotation2d rotation);
    public void resetpivot(Rotation2d resetAngle);
    public void disablepivot();

    public void setrollerRotation(Rotation2d rotation);
    public void resetroller(Rotation2d resetAngle);
    public void disableroller();
}
