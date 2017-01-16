package org.firstinspires.ftc.teamcode;
/**
 * Created by apurcell on 1/15/17.
 */
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a WEN-D.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */

public class HardwareWenD {

    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;

    public I2cDeviceSynch frontLeftRangeDevice = null;
    public I2cDeviceSynch frontCenterRangeDevice = null;
    public I2cDeviceSynch frontRightRangeDevice = null;
    public I2cDeviceSynch rightCenterRangeDevice = null;
    public I2cDeviceSynch backCenterRangeDevice = null;
    public I2cDeviceSynch leftCenterRangeDevice = null;

    public ModernRoboticsI2cRangeSensor frontLeftRangeSensor = null;
    public ModernRoboticsI2cRangeSensor frontCenterRangeSensor = null;
    public ModernRoboticsI2cRangeSensor frontRightRangeSensor = null;
    public ModernRoboticsI2cRangeSensor rightCenterRangeSensor = null;
    public ModernRoboticsI2cRangeSensor backCenterRangeSensor = null;
    public ModernRoboticsI2cRangeSensor leftCenterRangeSensor = null;

    public final static I2cAddr FRONT_LEFT_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x2a);
    public final static I2cAddr FRONT_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x2c);
    public final static I2cAddr FRONT_RIGHT_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x2e);
    public final static I2cAddr RIGHT_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x22);
    public final static I2cAddr BACK_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x24);
    public final static I2cAddr LEFT_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x28);

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareWenD(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run with encoders and reset encoders.
        resetMotorEncoders();

        //Init 6 Range Sensors
        frontLeftRangeDevice = (I2cDeviceSynch)ahwMap.get("fl");
        frontLeftRangeSensor = new ModernRoboticsI2cRangeSensor(frontLeftRangeDevice);
        frontLeftRangeSensor.setI2cAddress(FRONT_LEFT_RANGE_SENSOR_ADDRESS);
        frontLeftRangeDevice.engage();

        frontCenterRangeDevice = (I2cDeviceSynch)ahwMap.get("fc");
        frontCenterRangeSensor = new ModernRoboticsI2cRangeSensor(frontCenterRangeDevice);
        frontCenterRangeSensor.setI2cAddress(FRONT_CENTER_RANGE_SENSOR_ADDRESS);
        frontCenterRangeDevice.engage();

        frontRightRangeDevice = (I2cDeviceSynch)ahwMap.get("fr");
        frontRightRangeSensor = new ModernRoboticsI2cRangeSensor(frontRightRangeDevice);
        frontRightRangeDevice.disengage();
        frontRightRangeSensor.setI2cAddress(FRONT_RIGHT_RANGE_SENSOR_ADDRESS);
        frontRightRangeDevice.engage();

        rightCenterRangeDevice = (I2cDeviceSynch)ahwMap.get("rc");
        rightCenterRangeSensor = new ModernRoboticsI2cRangeSensor(rightCenterRangeDevice);
        rightCenterRangeSensor.setI2cAddress(RIGHT_CENTER_RANGE_SENSOR_ADDRESS);
        rightCenterRangeDevice.engage();

        backCenterRangeDevice = (I2cDeviceSynch)ahwMap.get("bc");
        backCenterRangeSensor = new ModernRoboticsI2cRangeSensor(backCenterRangeDevice);
        backCenterRangeSensor.setI2cAddress(BACK_CENTER_RANGE_SENSOR_ADDRESS);
        backCenterRangeDevice.engage();

        leftCenterRangeDevice = (I2cDeviceSynch)ahwMap.get("lc");
        leftCenterRangeSensor = new ModernRoboticsI2cRangeSensor(leftCenterRangeDevice);
        leftCenterRangeSensor.setI2cAddress(LEFT_CENTER_RANGE_SENSOR_ADDRESS);
        leftCenterRangeDevice.engage();
    }

    public void resetMotorEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

