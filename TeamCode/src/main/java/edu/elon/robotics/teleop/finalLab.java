package edu.elon.robotics.teleop;
/**
 * A simple teleop OpMode for drive the PushBot.
 * + Arm Controls.
 *
 * @author J. Hollingsworth
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import edu.elon.robotics.RobotHardware;


@TeleOp(name = "Final Lab Arm Driving", group = "TeleOp")
public class finalLab extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap);

        // move the arm to a known location
        initializeArm();

        waitForStart();

        while (opModeIsActive()) {
            stickDriving();
            controlArm();
            controlHand();
            boxAutomation();

            /* telemetry */
            showDriveMotorEncoders();
            telemetry.update();
        }
    }

    private void controlArm() {
        double armPower = 0.0;

        // drive the arm up/down
        if (gamepad1.dpad_up && robot.motorArm.getCurrentPosition() < robot.ARM_MAX_HEIGHT) {
            armPower = robot.ARM_POWER_UP;
        } else if (gamepad1.dpad_down && robot.touchSensor.getState()) {
            armPower = robot.ARM_POWER_DOWN;
        }

        // reset the encoder when we touch the button
        if (!robot.touchSensor.getState()) {
            robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // set the arm power
        robot.motorArm.setPower(armPower);

        // show information to the user
        telemetry.addData("arm power", armPower);
        telemetry.addData("arm ticks", robot.motorArm.getCurrentPosition());
    }

    // starting positions
    private double gripperPos = RobotHardware.GRIPPER_FULLY_OPEN;
    private double wristPos = RobotHardware.WRIST_PICKUP_POS;

    // used for toggling
    private boolean wasAPressed = false;
    private boolean wasBPressed = false;
    private boolean wasXPressed = false;
    private boolean wasYPressed = false;

    private boolean wasRightPressed = false;
    private boolean wasLeftPressed = false;

    private void controlHand() {
        // open/close the gripper
        if (gamepad1.b && !wasBPressed) {
            gripperPos = robot.GRIPPER_FULLY_OPEN;
        } else if (gamepad1.x && !wasXPressed) {
            gripperPos = robot.GRIPPER_FULLY_CLOSED;
        }

        // move the wrist up/down
        if (gamepad1.y && !wasYPressed) {
            wristPos += robot.WRIST_INCREMENT;
        } else if (gamepad1.a && !wasAPressed) {
            wristPos -= robot.WRIST_INCREMENT;
        }else if (gamepad1.left_bumper && !wasLeftPressed) {
            wristPos = robot.WRIST_PICKUP_POS;
        }

        // remember button presses
        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;
        wasXPressed = gamepad1.x;
        wasYPressed = gamepad1.y;
        wasLeftPressed=gamepad1.left_bumper;

        // limit the servo to possible gripper positions
//        gripperPos = Range.clip(gripperPos, robot.GRIPPER_FULLY_CLOSED, robot.GRIPPER_FULLY_OPEN);
//        wristPos = Range.clip(wristPos, robot.WRIST_FULLY_DOWN, robot.WRIST_FULLY_UP);

        // set the servo positions
        robot.servoGripper.setPosition(gripperPos);
        robot.servoWrist.setPosition(wristPos);

        // telemetry
        telemetry.addData("gripper pos", gripperPos);
        telemetry.addData("wrist pos", wristPos);
    }

    int redHeight= 10;
    int heightBlue=10;
    int strafeBlue=64;
    int strafeRed=50;
    int driveBlue=24;
    int driveRed=55;
    private void boxAutomation(){
        if(gamepad1.right_bumper && !wasRightPressed)
        {
            robot.servoWrist.setPosition(robot.WRIST_START_POS);
            robot.motorArm.setPower(robot.ARM_INIT_POWER);
            while (robot.touchSensor.getState()) {
                // do nothing -- waiting for a button press
            }

            robot.motorArm.setPower(0);

            // reset the encoder
            robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            int boxAlpha = robot.colorSensor.alpha();
            System.out.println("boxAlpha"+ boxAlpha);
            telemetry.addData("color Sensor",boxAlpha);
            // boxAlpha red is 220
            // boxAlpha blue is 197
            if(boxAlpha<200){ //blue
                // set height for motorArm

                while(Math.abs(robot.motorArm.getCurrentPosition()) < robot.ARM_MAX_HEIGHT && opModeIsActive()) {
                    robot.motorArm.setPower(robot.ARM_POWER_UP);
                }
                robot.motorArm.setPower(0);

                // drive to the box strafe then drive forward
                driveDistance(0,strafeBlue,.5);
                driveDistance(driveBlue,0,.5);
                robot.servoWrist.setPosition(.3);
                // open gripper
                robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
                driveDistance(driveBlue,0,-.5);
                driveDistance(0,strafeBlue,-.5);
            }
            else{
                while(Math.abs(robot.motorArm.getCurrentPosition()) < robot.ARM_MAX_HEIGHT && opModeIsActive()) {
                    robot.motorArm.setPower(robot.ARM_POWER_UP);
                }
                robot.motorArm.setPower(0);

                driveDistance(0,strafeRed,-0.5);
                driveDistance(driveRed,0,0.5);
                robot.servoWrist.setPosition(.3);
                // open gripper
                robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
                driveDistance(driveRed,0,-0.5);
                driveDistance(0,strafeRed,0.5);

            }


        }

        wasAPressed = gamepad1.right_bumper;

    }

    private void initializeArm() {

        // move the servo to known good positions
        robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        robot.servoWrist.setPosition(robot.WRIST_START_POS);

        // let the servos get to their position before moving the arm
        sleep(500);

        // tell the user that the arm is initializing
        telemetry.addData("ARM", "is initializing");
        telemetry.update();

        // initialize the arm
        robot.motorArm.setPower(robot.ARM_INIT_POWER);
        while (robot.touchSensor.getState()) {
            // do nothing -- waiting for a button press
        }

        robot.motorArm.setPower(0);

        // reset the encoder
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // tell the user we are done
        telemetry.addData("ARM", "initialization complete");
        telemetry.update();
    } // change how far down arm goes

    /************************************************************************
     * Drive the robot using the left/right joysticks.
     ************************************************************************/
    public void stickDriving() {
        /*
         * Read the gamepad joysticks and use that information
         * to drive the robot.
         */
        double drive  = -gamepad1.left_stick_y/2;
        double strafe = gamepad1.left_stick_x/2;
        double turn   = gamepad1.right_stick_x/2;

        // call startMove to move the robot
        robot.startMove(drive, strafe, turn, 1.0);
    }

    /************************************************************************
     * Telemetry - messages that show up on the Driver Station
     ************************************************************************/
    private void showDriveMotorEncoders() {
        telemetry.addData(" motorLeft: ", robot.motorLeft.getCurrentPosition());
        telemetry.addData("motorRight: ", robot.motorRight.getCurrentPosition());
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
}