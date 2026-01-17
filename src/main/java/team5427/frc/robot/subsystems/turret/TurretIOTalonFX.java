package team5427.frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.sql.rowset.BaseRowSet;

import org.checkerframework.checker.units.qual.C;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.lib.motors.SteelTalonFX;

public class TurretIOTalonFX implements TurretIO {
    
    private SteelTalonFX pivotMotor;
    private SteelTalonFX rollerMotor;

    private StatusSignal<Angle> pivotMotorAngle;
    private StatusSignal<AngularVelocity> pivotMotorAngularVelocity;
    private StatusSignal<AngularAcceleration> pivotMotorAngularAcceleration;
    private LinearVelocity pivotMotorLinearVelocity;
    private LinearAcceleration pivotMotorLinearAcceleration;
    private StatusSignal<Temperature> pivotMotorTemperature;
    private StatusSignal<Current> pivotMotorCurrent;
    private StatusSignal<Voltage> pivotMotorVoltage;

    private boolean pivotMotorIsConnected = false;

    private StatusSignal<Angle> rollerMotorAngle;
    private StatusSignal<Temperature> rollerMotorTemperature;
    private StatusSignal<Current> rollerMotorCurrent;
    private StatusSignal<Voltage> rollerMotorVoltage;

    private boolean rollerMotorIsConnected = false;

    public TurretIOTalonFX(){
        pivotMotor = new SteelTalonFX(null);

        pivotMotorAngle = pivotMotor.getTalonFX().getPosition();

        pivotMotorTemperature = pivotMotor.getTalonFX().getDeviceTemp();
        pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent();
        pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage();
        pivotMotorIsConnected = pivotMotor.getTalonFX().isConnected();

        rollerMotor = new SteelTalonFX(null);

        rollerMotorAngle = rollerMotor.getTalonFX().getPosition();

        rollerMotorTemperature = rollerMotor.getTalonFX().getDeviceTemp();
        rollerMotorCurrent = rollerMotor.getTalonFX().getStatorCurrent();
        rollerMotorVoltage = rollerMotor.getTalonFX().getMotorVoltage();
        rollerMotorIsConnected = rollerMotor.getTalonFX().isConnected();

    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.pivotMotorIsConnected = pivotMotor.getTalonFX().isConnected();


        BaseStatusSignal.refreshAll(pivotMotorAngle, pivotMotorAngularVelocity, pivotMotorAngularAcceleration, pivotMotorTemperature, pivotMotorVoltage, pivotMotorCurrent);


        inputs.pivotMotorAngle = Rotation2d.fromRadians(pivotMotor.getTalonFX().getPosition().getValue().magnitude());
        inputs.pivotMotorTemperature = pivotMotor.getTalonFX().getDeviceTemp().getValue();
        inputs.pivotMotorCurrent = pivotMotor.getTalonFX().getStatorCurrent().getValue();
        inputs.pivotMotorVoltage = pivotMotor.getTalonFX().getMotorVoltage().getValue();


        inputs.rollerMotorIsConnected = rollerMotor.getTalonFX().isConnected();

        BaseStatusSignal.refreshAll(rollerMotorAngle,rollerMotorTemperature, rollerMotorVoltage, rollerMotorCurrent);

        inputs.rollerMotorAngle = Rotation2d.fromRadians(rollerMotor.getTalonFX().getPosition().getValue().magnitude());
        inputs.rollerMotorTemperature = rollerMotor.getTalonFX().getDeviceTemp().getValue();
        inputs.rollerMotorCurrent = rollerMotor.getTalonFX().getStatorCurrent().getValue();
        inputs.rollerMotorVoltage = rollerMotor.getTalonFX().getMotorVoltage().getValue();


        inputs.rollerMotorIsConnected = rollerMotor.getTalonFX().isConnected();

    }

    @Override
    public void setpivotRotation(Rotation2d rotation) {
        pivotMotor.setSetpoint(rotation);
    }


    @Override
    public void resetpivot(Rotation2d
     resetAngle) {
        pivotMotor.setSetpoint(resetAngle);
    }
    
    @Override
    public void disablepivot(){
        pivotMotor.getTalonFX().disable();
    }

    @Override
    public void setrollerRotation(Rotation2d rotation) {
        rollerMotor.setSetpoint(rotation);
    }

    @Override
    public void resetroller(Rotation2d resetAngle) {
        rollerMotor.setSetpoint(resetAngle);
    }

    @Override
    public void disableroller() {
        rollerMotor.getTalonFX().disable();
    }

}
