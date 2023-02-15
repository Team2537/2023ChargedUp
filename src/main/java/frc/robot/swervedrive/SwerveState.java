package frc.robot.swervedrive;

/**
 * This class represents a state of a swerve module, which consists of a speed and an angle.
 */
public class SwerveState {
    private double m_angleRad;
    private double m_speedMPS;

    public SwerveState(double angleRad, double speedMPS) {
        m_angleRad = angleRad;
        m_speedMPS = speedMPS;
    }

    public double getSpeed() {
        return m_speedMPS;
    }

    public double getAngle() {
        return m_angleRad;
    }

    public void invert() {
        m_speedMPS *= -1;
    }

    public void setAngle(double angleRad) {
        m_angleRad = angleRad;
    }
}
