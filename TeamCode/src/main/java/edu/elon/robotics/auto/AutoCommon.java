package edu.elon.robotics.auto;

/**
 * General autonomous methods.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import edu.elon.robotics.RobotHardware;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;

    // how many degrees to adjust the requested degree angle by
    private final double ANGLE_OVERSHOOT = 3.0;

    // slow power of the motor for the final part of the turn
    private final double TURN_ENDING_POWER = 0.2;

    // number of degrees that will be done using the slow power
    private final double SLOW_DOWN_DEGREES = 15;

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
    protected void driveHeading(double distance, double heading, double maxPower){
        robot.resetDriveEncoders();
        double distanceForward = Math.sin(Math.abs(heading)) * Math.abs(distance);
        double distanceSide= Math.cos(Math.abs(heading)) * Math.abs(distance);

        if(heading < 0){
            while((Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDistanceToTicks(distanceForward)
                    || Math.abs(robot.motorAux.getCurrentPosition()) < robot.convertDistanceToTicks(distanceSide))&& opModeIsActive()) {
                robot.startMove(-maxPower, maxPower/2,0,0);
            }
        }
        if(heading >=0){
            while((Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDistanceToTicks(distanceForward)
                    || Math.abs(robot.motorAux.getCurrentPosition()) < robot.convertDistanceToTicks(distanceSide))&& opModeIsActive()) {
                robot.startMove(maxPower, maxPower/2,0,0);
            }
        }

    }

    protected void turnAngle(double degrees, double maxPower) {
        robot.resetDriveEncoders();
        while(Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDegreesToTicks(degrees) && opModeIsActive()) {
            robot.startMove(0,0,maxPower,0);
        }
        robot.startMove(0,0,0,0);

    }

    protected void driveDistance(double cmForward, double cmSide, double maxPower) {
        robot.resetDriveEncoders();
        if(cmForward != 0 && cmSide !=0){
            while((Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDistanceToTicks(cmForward)
                    || Math.abs(robot.motorAux.getCurrentPosition()) < robot.convertDistanceToTicks(cmSide))&& opModeIsActive()) {
                robot.startMove(maxPower, maxPower,0,0);
            }
        }
        if(cmForward == 0 && cmSide !=0){
            while(Math.abs(robot.motorAux.getCurrentPosition()) < robot.convertDistanceToTicks(cmSide) && opModeIsActive()) {
                robot.startMove(0, maxPower,0,0);
            }
        }
        if(cmForward != 0 && cmSide ==0){
            while(Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDistanceToTicks(cmForward) && opModeIsActive()) {
                robot.startMove(maxPower, 0,0,0);
            }
        }
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

    protected double getHeading() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = robot.imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

    protected void turnIMU(double degrees, double power) {
        //reset the yaw angle
        robot.imu.resetYaw();
        //start the motors turning the robot
        //wait until the heading of the robot matches the desired heading
        if(degrees<0) {
            while((Math.abs(degrees)-ANGLE_OVERSHOOT-SLOW_DOWN_DEGREES) >= Math.abs(getHeading()) && opModeIsActive() && getHeading()<1) {
                robot.startMove(0,0,power,0);
            }
            while((Math.abs(degrees)-ANGLE_OVERSHOOT)>=Math.abs(getHeading())&& opModeIsActive() && getHeading()<1){
                robot.startMove(0,0,TURN_ENDING_POWER,0);
            }
        }
        else {
            while(Math.abs(degrees-ANGLE_OVERSHOOT-SLOW_DOWN_DEGREES) >= Math.abs(getHeading()) && opModeIsActive() && getHeading()>-1) {
                robot.startMove(0,0,-power,0);

            }
            while(Math.abs(degrees-ANGLE_OVERSHOOT)>=Math.abs(getHeading())&& opModeIsActive() && getHeading()>-1){
                robot.startMove(0,0,-TURN_ENDING_POWER,0);
            }
        }
        robot.startMove(0,0,0,0);

    }


    protected void driveIMU(double cm, double power){
        robot.imu.resetYaw();
        double heading = getHeading();
        //negative yaw means turn aux clockwise
        //positive yaw means turn aux counter-clockwise
        //while yaw is not zero, add to it to make it zero
        while((Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDistanceToTicks(cm))){
            robot.motorLeft.setPower(power*.58);
            robot.motorRight.setPower(power*-.58);
            robot.motorAux.setPower(.1*getHeading());
        }

    }

    protected void touchMe(double power) {
        while(robot.touchSensor.getState()) {
            robot.startMove(power,0,0,0);
        }
        robot.startMove(0,0,0,0);
    }

    protected void driveToCalibrateLightSensor(double power) {
        int smallest=10000;
        int largest=0;
        while((Math.abs(robot.motorLeft.getCurrentPosition()) < robot.convertDistanceToTicks(20))){
            robot.startMove(power,0,0,0);
            System.out.println("brightness"+ robot.colorSensor.alpha());
            if(robot.colorSensor.alpha()<smallest){
                smallest=robot.colorSensor.alpha();
            }
            else if(robot.colorSensor.alpha()>largest){
                largest=robot.colorSensor.alpha();
            }
        }
        robot.startMove(0,0,0,0);
        // brightness smallest100brightness largest3660
        System.out.println("brightness smallest"+ smallest+"brightness largest"+largest);

        robot.maxBrightness=largest;
        robot.minBrightness=smallest;
    }


    protected void countWhiteLines(double power){
        robot.resetDriveEncoders();
        driveToCalibrateLightSensor(power);
        int numLines=0;
        boolean count=true;
        while(robot.touchSensor.getState()){
            System.out.println("alpha is currently"+robot.colorSensor.alpha());
            robot.startMove(power,0,0,0);

            if(robot.colorSensor.alpha() >= robot.maxBrightness-500 && count){
                System.out.println("lines detected at"+robot.colorSensor.alpha());
                numLines++;
                count=false;
            }
            else if (robot.colorSensor.alpha()<= robot.maxBrightness-700){
                count=true;
                System.out.println("countreset");
            }

        }

        double encoderValue=robot.motorLeft.getCurrentPosition();
        double cmFromEncoders=robot.convertTicksToDistance(encoderValue);
        System.out.println("CM TRACKED"+cmFromEncoders);

        telemetry.addData("cm tracked",cmFromEncoders);
        telemetry.addData("number of lines ",numLines);
        telemetry.update();

        driveDistance(cmFromEncoders,0,-.2);


        System.out.println("number of lines detected"+numLines);
        sleep(10000);
    }

}
