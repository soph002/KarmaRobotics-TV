package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="Drive Test 10/25", group = "labs")

public class DriveTest extends AutoCommon{
    @Override
    public void runOpMode() throws InterruptedException {
         /*
            When run, this class will drive or turn when the Y button is pressed on a gamepad.
            It is set to use driveIMU to drive forward 310 cm
            If you prefer to use the driveDistance method, change the Boolean below to false.

            It is set to use turnIMU for the turns
            If you prefer to use the turnAngle method, change the Boolean below to false.
        */
        boolean useIMUtoDrive = true;
        boolean useIMUtoTurn = true;

        super.runOpMode();
        waitForStart();

        int test = 0;
        while(opModeIsActive()) {
            if(gamepad1.y) {
                test += 1;
                if(test <= 3) {
                    //Drive Forward 310 cm
                    if(useIMUtoDrive)
                        driveIMU(310,.3);
                    else
                        driveDistance(310,0,.3);
                    sleep(2000);
                }
                if(test == 4) {
                    // Turn 90 degrees
                    if(useIMUtoTurn)
                        turnIMU(90, .3);
                    else
                        turnAngle(90, .3);
                    sleep(2000);
                }
                if(test == 5) {
                    // Turn -125 degrees
                    if(useIMUtoTurn)
                        turnIMU(-125, .3);
                    else
                        turnAngle(-125, .3);
                    sleep(2000);
                }
                if(test == 6) {
                    // Turn 180 degrees
                    if(useIMUtoTurn)
                        turnIMU(180, .3);
                    else
                        turnAngle(180, .3);
                    sleep(2000);
                }
            }
        }
    }
}