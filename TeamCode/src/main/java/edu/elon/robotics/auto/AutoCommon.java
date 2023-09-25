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

    protected void rampingForDrive(double power){
        double drive=power;
        int strafe=0;
        int turn=0;

        robot.resetDriveEncoders();
        double requestPower=drive+strafe+turn;
        double RAMP_TICKS = 1000;
        double CONSTANT_TICKS = 2000;
        double BASE_POWER = .4;
        double curPower=BASE_POWER;

        robot.motorLeft.setPower(BASE_POWER);
        robot.motorRight.setPower(-BASE_POWER);
        robot.motorAux.setPower(0);

        while(Math.abs(robot.motorLeft.getCurrentPosition()) < RAMP_TICKS && curPower<=requestPower && opModeIsActive()){
            robot.motorLeft.setPower(curPower);
            robot.motorRight.setPower(-curPower);
            robot.motorAux.setPower(0);
            curPower=BASE_POWER+(Math.abs(robot.motorLeft.getCurrentPosition())/RAMP_TICKS * (requestPower-BASE_POWER));
        }

        while(Math.abs(robot.motorLeft.getCurrentPosition()) < CONSTANT_TICKS && opModeIsActive())
        {
            double powerLeft= 0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
            double powerRight= -0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
            double powerAux= -2.0 * strafe / 3.0 + turn / 3.0;

            double scale = Math.max(Math.max(Math.max(1.0, Math.abs(powerLeft)), Math.abs(powerRight)), Math.abs(powerAux));

            powerLeft=powerLeft/scale;
            powerRight=powerRight/scale;
            powerAux=powerAux/scale;
            /*
             * Apply the power to the motors.
             */
            robot.motorLeft.setPower(powerLeft);
            robot.motorRight.setPower(powerRight);
            robot.motorAux.setPower(powerAux);
        }
        int endingPosition=Math.abs(robot.motorLeft.getCurrentPosition());

        // the robot goes slightly backwards at the end
        // this is due to most likely due to the robot getting so close to 0
        // but not reaching the desired ramp ticks so the motors does not then get set to 0
        while(Math.abs(robot.motorLeft.getCurrentPosition()) < RAMP_TICKS+endingPosition && opModeIsActive()){
            robot.motorLeft.setPower(curPower);
            robot.motorRight.setPower(-curPower);
            robot.motorAux.setPower(0);
            curPower=requestPower-(Math.abs(robot.motorLeft.getCurrentPosition())/RAMP_TICKS * (requestPower-BASE_POWER));
        }

        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
        robot.motorAux.setPower(0);
    }

}
