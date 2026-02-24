package frc.robot.chassis.commands;

// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.vision.ObjectPose;

public class AutoIntake extends Command {
    private Chassis chassis;
    // private boolean finish = false;
    private ObjectPose objectDetection;
    // private Pose2d targetPoseFromRobot = Pose2d.kZero;
    private double maxVel = 2;
    // private double maxOmega = Math.toRadians(180);
    // private Translation2d diffVector = Translation2d.kZero;
    private Translation2d robotToIntake = new Translation2d(-0.2, 0);

    public AutoIntake(Chassis chassis, ObjectPose objectDetection) {
        this.chassis = chassis;
        this.objectDetection = objectDetection;
        addRequirements(chassis);
    }

    // private double calcTimeToRotate(double angle) {
        // return 2 * (Math.sqrt(Math.abs(angle) / Math.toRadians(540)));
    // }

    @Override
    public void execute() {

        Translation2d diffVector = objectDetection.getRobotToObject().minus(robotToIntake);
        // double time = calcTimeToRotate(diffVector.getAngle().getRadians());

        // double velocity = Math.min((diffVector.getNorm() - 0.2) / time, maxVel);
        double velocity = Math.min(diffVector.getNorm() * 2, maxVel);
        frc.demacia.utils.log.LogManager.log("diff vector: " + diffVector);

        // System.out.println("drive velocity= " + velocity);
        ChassisSpeeds speeds = new ChassisSpeeds(-velocity * diffVector.getAngle().getCos(),
                -velocity * diffVector.getAngle().getSin(),
                0);

        chassis.setRobotRelVelocities(speeds);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.setVelocities(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return objectDetection.getRobotToObject().minus(robotToIntake).getNorm() < 0.03;
    }

}
