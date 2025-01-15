package frc.robot.subsystems.drivetrain;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase{
    SwerveDrive swerveDrive; 

    public Drivetrain() throws IOException{
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(6.0);
    }

    public void configureDrive(){
        swerveDrive.setHeadingCorrection(
            false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(
            false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
        // simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(
            true, false,
            0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
        // coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(
            false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders
        // periodically when they are not moving.
        // swerveDrive.setChassisDiscretization(true, false, Constants.DrivetrainConstants.kSecondOrderKinematicsDt);
        swerveDrive
            .pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder
        // and push the offsets onto it. Throws warning if not possible
    }

}

