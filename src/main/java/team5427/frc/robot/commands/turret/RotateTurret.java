package team5427.frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Meters;

import java.util.Collection;

import org.checkerframework.checker.initialization.qual.Initialized;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.subsystems.turret.TurretSubsystem;
import team5427.frc.robot.subsystems.vision.VisionSubsystem;

public class RotateTurret extends Command{
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;

    private Pose2d targetPose;

    public RotateTurret(){
        turretSubsystem = TurretSubsystem.getInstance();
        visionSubsystem = VisionSubsystem.getInstance();
        addRequirements((Subsystem) turretSubsystem, (Subsystem) visionSubsystem);
    }

    @Override
    public void initialize() {
        Rotation2d camera1XAngle = visionSubsystem.getTargetX(0);
        Rotation2d camera2XAngle = visionSubsystem.getTargetX(1);

        Rotation2d pivotAngle = new Rotation2d((camera1XAngle.getRadians()+camera2XAngle.getRadians())/2.0);
        turretSubsystem.setpivotRotation(pivotAngle);

        Distance distanceToTag = Meters.of(Math.cos(pivotAngle.getRadians())*RobotPose.getInstance().getAdaptivePose().getY());

        Distance hubHeight = Meters.of(1.8288);

        Rotation2d rollerAngle = Rotation2d.fromRadians(Math.asin(hubHeight.magnitude()/distanceToTag.magnitude()));

        turretSubsystem.setrollerRotation(rollerAngle);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
