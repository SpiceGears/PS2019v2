package frc.team5883.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.team5883.robot.Robot;

public class ResetEncoderPos extends Command {

    public ResetEncoderPos() {

        requires(Robot.driveTrain);

    }

    protected void initialize() {

        //Robot.driveTrain.changeToPercentVBus();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

}
