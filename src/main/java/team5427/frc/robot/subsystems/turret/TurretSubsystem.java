package team5427.frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import team5427.frc.robot.subsystems.shooter.ShooterIOTalonFX;

public class TurretSubsystem extends SubsystemBase {
  private Rotation2d currentAngle;
  

  private TurretIOTalonFX io = new TurretIOTalonFX();
  private TurretIOInputsAutoLogged inputsAutoLogged;

  public static TurretSubsystem m_instance;

  private TurretSubsystem() {
    // Constructor for IntakeSubsystem
    // Initialize any necessary components or configurations here

    inputsAutoLogged = new TurretIOInputsAutoLogged();
    switch (Constants.currentMode) {
      case REAL:
        io = new TurretIOTalonFX();
        break;
      case SIM:
        io = null;
        break;
      default:
        break;}
    }

  public static TurretSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new TurretSubsystem();
    }
    return m_instance;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputsAutoLogged);
    Logger.processInputs("Turret/Inputs", inputsAutoLogged);
    log();
  }

    public void setpivotRotation(Rotation2d rotation) {
        io.setpivotRotation(rotation);
    }


    public void resetpivot(Rotation2d resetAngle) {
        io.resetpivot(resetAngle);
    }
    
    public void disablepivot(){
        io.disablepivot();
    }

    public void setrollerRotation(Rotation2d rotation) {
        io.setpivotRotation(rotation);
    }


    public void resetroller(Rotation2d resetAngle) {
        io.resetpivot(resetAngle);
    }
    
    public void disableroller(){
        io.disableroller();
    }

  public void log() {
    Logger.recordOutput("Turret/TurretAngle", currentAngle);
  }

}
