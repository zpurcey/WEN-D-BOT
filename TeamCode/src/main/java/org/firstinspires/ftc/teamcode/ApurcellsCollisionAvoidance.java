package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="WEN-D", group="Pushbot")

public class ApurcellsCollisionAvoidance extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareWenD robot = new HardwareWenD();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.

    private ElapsedTime runtime = new ElapsedTime();

    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    static final double     COUNTS_PER_MOTOR_REV = 1478.4;          //1478.4hmm seems double what it should be
    static final double     DRIVE_GEAR_REDUCTION = 1.666666667;     // (driven gear tooth count)/(driving gear tooth count) 40/24
    static final double     WHEEL_DIAMETER_CM = 9.2;                // For figuring circumference
    static final double     TUNING_MULTIPLIER = 1.4;
    static final double     COUNTS_PER_CM = ( (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415) * TUNING_MULTIPLIER);

    static final double     FORWARD_SPEED = 0.6; //0.6;
    static final double     TURN_SPEED    = 0.5; //0.5;
    static final double     STOP_SPEED    = 0;


    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.update();
        while (opModeIsActive()) {

            addRangeSensorTelemtry();
            telemetry.update();

            // Step 1:  Drive forward till about to hit something
            robot.leftMotor.setPower(FORWARD_SPEED);
            robot.rightMotor.setPower(FORWARD_SPEED);
            runtime.reset();

            while (opModeIsActive() && safeToMoveForward() ) {
                adjustHeading();
                addRangeSensorTelemtry();
                addMotorEncoderTelemtry();
                telemetry.update();
            }

            driveBack(1);

            double obstacleDistanceDifference = robot.frontLeftRangeSensor.getDistance(DistanceUnit.CM) - robot.frontRightRangeSensor.getDistance(DistanceUnit.CM);
            if (obstacleDistanceDifference < -5){
                turnRight();
            }
            else {
                turnLeft();
            }
        }
    }

    private void driveBack(double secsToDriveBack){
        // Step 2:  Drive Backwards for 1 Second
        robot.leftMotor.setPower(-FORWARD_SPEED);
        robot.rightMotor.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < secsToDriveBack) && safeToMoveBackward() ) {
            addRangeSensorTelemtry();
            telemetry.update();
        }
    }

    private void turnLeft() {
        // Step 3:  Spin right for .65 seconds
        robot.leftMotor.setPower(-TURN_SPEED);
        robot.rightMotor.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .65)) {
            addRangeSensorTelemtry();
            telemetry.update();
        }
    }

    private void turnRight() {
        // Step 3:  Spin right for .65 seconds
        robot.leftMotor.setPower(TURN_SPEED);
        robot.rightMotor.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .65)) {
            addRangeSensorTelemtry();
            telemetry.update();
        }
    }

    private void adjustLeft(){
        // Step 3:  Spin left for .25 seconds
        robot.leftMotor.setPower(TURN_SPEED);
        //robot.rightMotor.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .25) && safeToMoveForward()) {
            addRangeSensorTelemtry();
            telemetry.update();
        }
        robot.leftMotor.setPower(FORWARD_SPEED);
    }

    private void adjustRight(){
        // Step 3:  Spin right for .25 seconds
        robot.rightMotor.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .25) && safeToMoveForward()) {
            addRangeSensorTelemtry();
            telemetry.update();
        }
        robot.rightMotor.setPower(FORWARD_SPEED);
    }

    private void adjustHeading(){

        //Getting to close to a wall? Steer away slightly
        if (robot.leftCenterRangeSensor.getDistance(DistanceUnit.CM) < 8){
            adjustRight();
        }
        if (robot.rightCenterRangeSensor.getDistance(DistanceUnit.CM) < 8){
            adjustLeft();
        }

        //Is there an obstacle coming? Try to steer towards a main path
        double obstacleDistanceDifference = robot.frontLeftRangeSensor.getDistance(DistanceUnit.CM) - robot.frontRightRangeSensor.getDistance(DistanceUnit.CM);
        if (obstacleDistanceDifference < -4){
            adjustRight();
        }

        if (obstacleDistanceDifference > 4){
            adjustLeft();
        }
    }

    private void addRangeSensorTelemtry(){
        telemetry.addData("Front Left Sensor Reads: ", robot.frontLeftRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Center Sensor Reads: ", robot.frontCenterRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Right Sensor Reads: ", robot.frontRightRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Center Sensor Reads: ", robot.rightCenterRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Center Sensor Reads: ", robot.backCenterRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Center Sensor Reads: ", robot.leftCenterRangeSensor.getDistance(DistanceUnit.CM));
    }

    private void addMotorEncoderTelemtry(){
        telemetry.addData("Left Motor Current Position: ", robot.leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Current Position: ", robot.rightMotor.getCurrentPosition());
        telemetry.addData("Left Motor Has Moved (cm): ", (robot.leftMotor.getCurrentPosition()/COUNTS_PER_CM) );
        telemetry.addData("Right Motor Has Moved (cm): ", (robot.rightMotor.getCurrentPosition()/COUNTS_PER_CM) );
        telemetry.addData("Counts per CM: ", COUNTS_PER_CM);
    }

    private boolean safeToMoveForward(){
        return safeToMoveForward(15);
    }

    private boolean safeToMoveBackward(){
        return safeToMoveBackward(15);
    }

    private boolean safeToMoveForward(int safeDistance){
        if( (robot.frontLeftRangeSensor.getDistance(DistanceUnit.CM) < safeDistance)
                || (robot.frontCenterRangeSensor.getDistance(DistanceUnit.CM) < safeDistance)
                || (robot.frontRightRangeSensor.getDistance(DistanceUnit.CM) < safeDistance) ) {
            return false;
        }
        return true;
    }

    private boolean safeToMoveBackward(int safeDistance){
        if( (robot.backCenterRangeSensor.getDistance(DistanceUnit.CM) < safeDistance) ) {
            return false;
        }
        return true;
    }
}
