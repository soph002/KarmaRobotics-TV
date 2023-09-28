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

        driveDistance(150, 0,.5);
        sleep(100);
        driveDistance(50,0,-.5);
        turnAngle(90,-.5);
        driveDistance(100,0,.5);
        turnAngle(70,.5);
        driveDistance(106.42, 0,-.5);
        turnAngle(250,-.5);
        driveDistance(63.6,0,.5);
        turnAngle(90,-.5);
        sleep(500);

    }

}
