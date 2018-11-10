package frc.team5883.autoModes;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5883.auto.MultiReversiblePathReader;
import frc.team5883.auto.PathEncoderFollower;

public class TestAuto extends CommandGroup {

    public TestAuto(){
        SmartDashboard.putNumber("TEST", 1);
        addSequential(new PathEncoderFollower("TestPaths/Smode3", 0.03));
        //addSequential(new MultiReversiblePathReader("TestPaths/Smode2", 20, MultiReversiblePathReader.Direction.COLLECTOR_FIRST, MultiReversiblePathReader.CSVReadDirection.TOP_TO_BOTTOM));
    }
}
