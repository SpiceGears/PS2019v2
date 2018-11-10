package frc.team5883.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team5883.autoModes.TestAuto;
import frc.team5883.subsystems.DriveTrain;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;


public class Robot extends IterativeRobot {

    Command autonomousCommand;
    SendableChooser<String> autoChooser;
    NetworkTable table;

    public static OI oi;
    public static DriveTrain driveTrain;
    public double debugowanie = 0;
    public UsbCamera camera;
    public void robotInit() {
        Constants.getConstants().setGameData("");
        table = NetworkTable.getTable("Cube");

        RobotMap.init();

        driveTrain = new DriveTrain();

        oi = new OI();

        autoChooser = new SendableChooser();
        autoChooser.addDefault("Default Program",  "DriveStright");
        autoChooser.addObject("Nothing1", "Nothing1");
        autoChooser.addObject("Test", "Test");
        SmartDashboard.putData("Autonomous mode chooser", autoChooser );
        Robot.driveTrain.encoderRight.reset();

        new Thread(() -> {
            camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(320, 240);
            camera.setFPS(15);
            camera.setBrightness(60);
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);

            Mat source = new Mat();
            Mat output = new Mat();

            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();

    }

    public void disabledInit(){

    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {



//        String gameData = null;
//        gameData = DriverStation.getInstance().getGameSpecificMessage();

//        int retries = 100;
//
//        while(gameData.length() < 2 && retries > 0) {
//            retries--;
//            try {
//                Thread.sleep(25);
//            } catch (InterruptedException e) {
//
//            }
//
//            gameData = DriverStation.getInstance().getGameSpecificMessage();
//
//        }
//        if(gameData.length() > 0) {
//            Constants.getConstants().setGameData(gameData);
//        }


        switch (autoChooser.getSelected()) {
            case "Nothing1":
                autonomousCommand = null;
                break;
            case "Test":
                autonomousCommand = new TestAuto();

            default:
                autonomousCommand = new TestAuto();
                break;
        }


        //autonomousCommand = new TestAuto();
        autonomousCommand.start();

    }


    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
       // Robot.driveTrain.updateAuto();

    }

    public void teleopInit() {
        driveTrain.resetSpeed();
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    public void teleopPeriodic() {

        //DASHBOARD
        double x = 0;
        x = table.getNumber("cube", 0);
        SmartDashboard.putNumber("X", x);

        Scheduler.getInstance().run();
    }

    public void testPeriodic() {
        LiveWindow.run();
    }

}