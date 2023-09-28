package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="Timed Drive", group="labs")
public class TimedDrive extends AutoCommon{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // driveForTime(0.3,2000);
        driveDistance(50, 50,.5);
        sleep(500);
//        sleep(500);
//        turnAngle(90, -.5);
//        sleep(500);
//        turnAngle(360, .5);
//        strafeForTime(0.7, 500);
//        turnForTime(1,500);
    }

}
