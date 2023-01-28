package frc.robot.swervedrive;

import static frc.robot.Constants.*;

public class SwerveKinematics {

    /**
     * Get the swerve states from measurements of the speed of movement of the frame
     * @param forward forward translation in m/s
     * @param strafe side to side translation in m/s
     * @param rotation angular velocity in rad/s
     * @return Array of swerve states in the order: front left, front right, back left, back right
     */
    public static SwerveState[] getSwerveStates(double forward, double strafe, double rotation) {
        double a = forward - rotation * WHEEL_BASE / 2;
        double b = forward + rotation * WHEEL_BASE / 2;
        double c = strafe - rotation * TRACK_WIDTH / 2;
        double d = strafe + rotation * TRACK_WIDTH / 2;
    
        double speedFL = Math.sqrt(b*b + d*d);
        double speedFR = Math.sqrt(b*b + c*c);
        double speedBL = Math.sqrt(a*a + d*d);
        double speedBR = Math.sqrt(a*a + c*c);
    
        double angleFL = Math.atan2(d, b) - Math.PI/2;
        double angleFR = Math.atan2(c, b) - Math.PI/2;
        double angleBL = Math.atan2(d, a) - Math.PI/2;
        double angleBR = Math.atan2(c, a) - Math.PI/2;
    
        return new SwerveState[] {
            new SwerveState(angleFL, speedFL),
            new SwerveState(angleFR, speedFR),
            new SwerveState(angleBL, speedBL),
            new SwerveState(angleBR, speedBR)
        };
    }
}
