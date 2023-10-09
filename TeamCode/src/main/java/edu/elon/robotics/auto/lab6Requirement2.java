package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="lab6 Req2", group="labs")
public class lab6Requirement2 extends AutoCommon{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // test driveIMU()
        driveIMU(300,.5);
    }

}
