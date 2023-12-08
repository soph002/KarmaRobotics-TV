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
//            boxAutomation();
            armInital();
            dropHighWall();
            dropLowWall();

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
    private boolean wasLeftTriggerPressed=false;
    private boolean wasRightTriggerPressed=false;

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
        }

        // remember button presses
        wasAPressed = gamepad1.a;
        wasBPressed = gamepad1.b;
        wasXPressed = gamepad1.x;
        wasYPressed = gamepad1.y;

        // limit the servo to possible gripper positions
        gripperPos = Range.clip(gripperPos, robot.GRIPPER_FULLY_CLOSED, robot.GRIPPER_FULLY_OPEN);
        wristPos = Range.clip(wristPos, robot.WRIST_FULLY_DOWN, robot.WRIST_FULLY_UP);

        // set the servo positions
        robot.servoGripper.setPosition(gripperPos);
        robot.servoWrist.setPosition(wristPos);

        // telemetry
        telemetry.addData("gripper pos", gripperPos);
        telemetry.addData("wrist pos", wristPos);
    }
    boolean fartherCourse=false;

    private void armInital(){
        if(gamepad1.left_bumper && !wasLeftPressed){
            robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
            robot.servoWrist.setPosition(robot.WRIST_START_POS);

            // let the servos get to their position before moving the arm
            sleep(500);
            // initialize the arm
            robot.motorArm.setPower(robot.ARM_POWER_DOWN);
            while (robot.touchSensor.getState()) {
                // do nothing -- waiting for a button press
            }

            robot.motorArm.setPower(0);
        }
        wasLeftPressed=gamepad1.left_bumper;
    }
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

            int boxBlue = robot.colorArm.blue();
            System.out.println("alpha"+robot.colorArm.alpha());
            System.out.println("blue"+robot.colorArm.blue());
            System.out.println("red"+robot.colorArm.red()+"g"+robot.colorArm.green());
            telemetry.addData("color Sensor",boxBlue);

            if(fartherCourse){
                int armTicksRed= 1007;
                double gripperRed=.15;
                int armTickBlue=292;
                double gripperBlue=.55;
                int strafeBlue=35;
                int strafeRed=62;
                int driveBlue=0;
                int driveRed=11;
                if(boxBlue>300){ //blue
                    // set height for motorArm
                    while(Math.abs(robot.motorArm.getCurrentPosition()) < armTickBlue && opModeIsActive()) {
                        robot.motorArm.setPower(robot.ARM_POWER_UP);
                    }
                    robot.motorArm.setPower(0);

                    // drive to the box strafe then drive forward
                    driveDistance(0,strafeBlue,.5);
                    driveDistance(driveBlue,0,.5);
                    sleep(500);
                    robot.servoWrist.setPosition(gripperBlue);
                    sleep(500);
                    // open gripper
                    robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
                    sleep(500);
                    driveDistance(driveBlue,0,-.5);
                    driveDistance(0,strafeBlue,-.5);
                }

                else{
                    while(Math.abs(robot.motorArm.getCurrentPosition()) < armTicksRed && opModeIsActive()) {
                        robot.motorArm.setPower(robot.ARM_POWER_UP);
                    }
                    robot.motorArm.setPower(0);

                    driveDistance(0,strafeRed,-0.5);
                    driveDistance(driveRed,0,0.5);
                    sleep(500);
                    robot.servoWrist.setPosition(gripperRed);
                    // open gripper
                    robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
                    sleep(500);
                    driveDistance(driveRed,0,-0.5);
                    driveDistance(0,strafeRed,0.5);
                }
            }
            else{
                int strafeBlue=69;
                int strafeRed=62;
                int driveBlue=15;
                int driveRed=45;
                int robotArmDroppingPositionLOW=268;

                if(boxBlue>300){ //blue
                    // set height for motorArm

                    while(Math.abs(robot.motorArm.getCurrentPosition()) < 872 && opModeIsActive()) {
                        robot.motorArm.setPower(robot.ARM_POWER_UP);
                    }
                    robot.motorArm.setPower(0);

                    // drive to the box strafe then drive forward
                    driveDistance(0,strafeBlue,.5);
                    driveDistance(driveBlue,0,.5);
                    sleep(500);
                    robot.servoWrist.setPosition(.2);
                    sleep(500);
                    // open gripper
                    robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
                    sleep(500);
                    driveDistance(driveBlue,0,-.5);
                    driveDistance(0,strafeBlue,-.5);
                }
                else{ //red
                    while(Math.abs(robot.motorArm.getCurrentPosition()) < robotArmDroppingPositionLOW && opModeIsActive()) {
                        robot.motorArm.setPower(robot.ARM_POWER_UP);
                    }
                    robot.motorArm.setPower(0);

                    driveDistance(0,strafeRed,-0.5);
                    driveDistance(driveRed,0,0.5);
                    sleep(500);
                    robot.servoWrist.setPosition(.54);
                    // open gripper
                    robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
                    sleep(500);
                    driveDistance(driveRed,0,-0.5);
                    driveDistance(0,strafeRed,0.5);

                }
            }


        }

        wasAPressed = gamepad1.right_bumper;

    }

    private void dropHighWall(){
        if(gamepad1.left_trigger >.1 && !wasLeftTriggerPressed){
            System.out.println("Moving arm");
            while(Math.abs(robot.motorArm.getCurrentPosition()) > 689 && opModeIsActive()) {
                System.out.println("Moving ARM MOVE");
                robot.motorArm.setPower(robot.ARM_POWER_DOWN);
            }
            while(Math.abs(robot.motorArm.getCurrentPosition()) < 689 && opModeIsActive()) {
                robot.motorArm.setPower(robot.ARM_POWER_UP);
            }
            robot.motorArm.setPower(0);
            robot.servoWrist.setPosition(.1);
            sleep(500);
            robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        }

        wasLeftTriggerPressed = gamepad1.left_trigger>.1;


    }

    private void dropLowWall(){
        if(gamepad1.right_trigger>.1 && !wasRightTriggerPressed){
            System.out.println("Moving arm low"+robot.motorArm.getCurrentPosition());
            robot.servoWrist.setPosition(.2); // may need to change
            System.out.println("Moving arm low wrist"+robot.servoWrist.getPosition());
            while(Math.abs(robot.motorArm.getCurrentPosition()) > 370 && opModeIsActive()) {
                robot.motorArm.setPower(robot.ARM_POWER_DOWN);
            }
            while(Math.abs(robot.motorArm.getCurrentPosition()) < 370 && opModeIsActive()) {
                robot.motorArm.setPower(robot.ARM_POWER_UP);
            }
            robot.motorArm.setPower(0);
            sleep(500);
            robot.servoGripper.setPosition(robot.GRIPPER_FULLY_OPEN);
        }

        wasRightTriggerPressed = gamepad1.right_trigger>.1;


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