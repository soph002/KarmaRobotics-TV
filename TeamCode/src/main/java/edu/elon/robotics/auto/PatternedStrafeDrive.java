package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Patterned Strafe Drive", group="labs")
public class PatternedStrafeDrive extends AutoCommon{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //drive forward 150cm
        //drive backward 50cm
        //turn counter-clockwise 90 degrees
        //drive forward 100cm
        //turn clockwise 70 degrees
        //drive backward 106.42cm
        //turn counter-clockwise 250 degrees
        //drive forward 63.6cm
        //turn counter-clockwise 90 degrees

        double speed = .4;
        int SLEEPTIME = 300;
        //driveDistance(36.39778, 100.002,-speed);
        //driveHeading(50,-20,.5);
        driveDistance(150, 0, speed);

        sleep(SLEEPTIME);
        driveDistance(50,0,-speed);
        sleep(SLEEPTIME);
        driveDistance(0,100,-speed);
        sleep(SLEEPTIME);

        driveHeading(80,-20,.5);

        driveDistance(0,63.6,speed);
        sleep(SLEEPTIME);

    }

}
