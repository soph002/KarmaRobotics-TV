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

    protected void rampingForDrive(double requestPower){
        robot.resetDriveEncoders();
        double RAMP_TICKS = 1000;
        double CONSTANT_TICKS = 2000;
        double BASE_POWER = .4;
        double curPower=BASE_POWER;

        robot.startMove(BASE_POWER,0,0,0);

        while(Math.abs(robot.motorLeft.getCurrentPosition()) < RAMP_TICKS && curPower<=requestPower && opModeIsActive()){
            robot.startMove(curPower,0,0,0);
            curPower=BASE_POWER+(Math.abs(robot.motorLeft.getCurrentPosition())/RAMP_TICKS * (requestPower-BASE_POWER));
        }

        while(Math.abs(robot.motorLeft.getCurrentPosition()) < CONSTANT_TICKS && opModeIsActive())
        {
            robot.startMove(requestPower,0,0,0);
        }
        int endingPosition=Math.abs(robot.motorLeft.getCurrentPosition());

        // the robot goes slightly backwards at the end
        // this is due to most likely due to the robot getting so close to 0
        // but not reaching the desired ramp ticks so the motors does not then get set to 0
        while(Math.abs(robot.motorLeft.getCurrentPosition()) < RAMP_TICKS+endingPosition && opModeIsActive()){
            robot.startMove(curPower,0,0,0);
            curPower=requestPower-(Math.abs(robot.motorLeft.getCurrentPosition())/RAMP_TICKS * (requestPower-BASE_POWER));
        }

        robot.startMove(0,0,0,0);
    }

}
