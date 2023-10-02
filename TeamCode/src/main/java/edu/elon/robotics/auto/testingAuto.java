package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="Testing Drive", group="labs")
public class testingAuto extends AutoCommon{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // driveForTime(0.3,2000);
        double speed=.3;
        driveDistance(0, 123,speed);
        sleep(900);
        turnAngle(90,-speed);
        sleep(300);
        driveDistance(123,0,speed);
//        sleep(500);
//        turnAngle(90, -.5);
//        sleep(500);
//        turnAngle(360, .5);
//        strafeForTime(0.7, 500);
//        turnForTime(1,500);
    }

}
