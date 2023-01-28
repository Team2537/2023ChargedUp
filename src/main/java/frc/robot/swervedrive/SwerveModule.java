package frc.robot.swervedrive;

public class SwerveModule {
    private final SteerMotor m_steerMotor;
    private final DriveMotor m_driveMotor;

    private SwerveState m_currentState;

    public SwerveModule(SteerMotor steerMotor, DriveMotor driveMotor) {
        m_steerMotor = steerMotor;
        m_driveMotor = driveMotor;

        m_currentState = new SwerveState(steerMotor.getPosition(), driveMotor.getVelocity());
    }

    public void setState(SwerveState desiredState) {
        optimizeState(desiredState);

        m_steerMotor.setDesiredState(desiredState.getAngle());
        m_driveMotor.setDesiredState(desiredState.getSpeed());
    }

    private double optimizeAngleRad(double angleRad) {
        angleRad %= (2.0*Math.PI); //will be between -2pi and 2pi exclusive
        if(angleRad > Math.PI) {
          angleRad-= (2.0*Math.PI);
        }
        if(angleRad <-Math.PI) {
          angleRad+= (2.0*Math.PI);
        }
        return angleRad;
    }

    private void optimizeState(SwerveState desiredState) {
        desiredState.setAngle(optimizeAngleRad(desiredState.getAngle()));

        double deltaTheta = desiredState.getAngle() - m_currentState.getAngle();
        if (deltaTheta > Math.PI / 2) {
            deltaTheta -= Math.PI;
            desiredState.invert();
        } else if (deltaTheta < -Math.PI / 2) {
            deltaTheta += Math.PI;
            desiredState.invert();
        }

        desiredState.setAngle(m_currentState.getAngle() + deltaTheta);
    }
}
