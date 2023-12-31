package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="Patterned Drive", group="labs")
public class PatternedDrive extends AutoCommon{

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

        driveDistance(150, 0, speed);
        int SLEEPTIME = 200;
        sleep(SLEEPTIME);
        driveDistance(50,0,-speed);
        sleep(SLEEPTIME);
        turnAngle(90,-speed);
        sleep(SLEEPTIME);
        driveDistance(100,0,speed);
//        driveDistance(0,100,-speed);
        sleep(SLEEPTIME);
        turnAngle(70,speed);
        sleep(SLEEPTIME);
        driveDistance(106.42, 0,-speed);
        sleep(SLEEPTIME);
        turnAngle(250,-speed);
        sleep(SLEEPTIME);
        driveDistance(63.6,0,speed);
        sleep(SLEEPTIME);
        turnAngle(90,-speed);
        sleep(500);

    }

}
