package team5427.frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;

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
        break;
    }
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

  public void disablepivot() {
    io.disablepivot();
  }

  public void setrollerRotation(Rotation2d rotation) {
    io.setpivotRotation(rotation);
  }

  public void resetroller(Rotation2d resetAngle) {
    io.resetpivot(resetAngle);
  }

  public void disableroller() {
    io.disableroller();
  }

  public void log() {
    Logger.recordOutput("Turret/TurretAngle", currentAngle);
  }
}
