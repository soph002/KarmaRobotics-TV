package edu.elon.robotics.auto;

/**
 * General autonomous methods.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.elon.robotics.RobotHardware;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);
    }

    protected void driveForTime(double power, long milliseconds){
        robot.startMove(power, 0,0,0);
        sleep(milliseconds);
        robot.startMove(0,0,0,0);
    }

    protected void strafeForTime(double power, long milliseconds){
        robot.startMove(0, power,0,0);
        sleep(milliseconds);
        robot.startMove(0,0,0,0);
    }

    protected void turnForTime(double power, long milliseconds){
        robot.startMove(0,0,power,0);
        sleep(milliseconds);
        robot.startMove(0,0,0,0);
    }
}
