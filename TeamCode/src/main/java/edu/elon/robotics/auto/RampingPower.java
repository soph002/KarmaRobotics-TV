package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Ramping Power", group="labs")
public class RampingPower extends AutoCommon{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        robot.resetDriveEncoders();
        driveForTime(0.8,2000);
    }
}
