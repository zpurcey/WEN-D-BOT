package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="RangeSensorTests", group="Pushbot")

public class RangeSensorTests extends OpMode {


    ModernRoboticsI2cRangeSensor frontLeftRangeSensor;
    I2cDeviceSynch frontLeftRangeDevice;

    ModernRoboticsI2cRangeSensor frontCenterRangeSensor;
    I2cDeviceSynch frontCenterRangeDevice;

    ModernRoboticsI2cRangeSensor frontRightRangeSensor;
    I2cDeviceSynch frontRightRangeDevice;

    ModernRoboticsI2cRangeSensor rightCenterRangeSensor;
    I2cDeviceSynch rightCenterRangeDevice;

    ModernRoboticsI2cRangeSensor backCenterRangeSensor;
    I2cDeviceSynch backCenterRangeDevice;

    ModernRoboticsI2cRangeSensor leftCenterRangeSensor;
    I2cDeviceSynch leftCenterRangeDevice;



    //ModernRoboticsI2cRangeSensor frontCenterRangeSensor;
    public final static I2cAddr FRONT_LEFT_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x2a);
    public final static I2cAddr FRONT_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x2c);
    public final static I2cAddr FRONT_RIGHT_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x2e);
    public final static I2cAddr RIGHT_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x22);
    public final static I2cAddr BACK_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x24);
    public final static I2cAddr LEFT_CENTER_RANGE_SENSOR_ADDRESS = I2cAddr.create8bit(0x28);

    public void init() {
        frontLeftRangeDevice = (I2cDeviceSynch)hardwareMap.get("fl");
        frontLeftRangeSensor = new ModernRoboticsI2cRangeSensor(frontLeftRangeDevice);
        frontLeftRangeSensor.setI2cAddress(FRONT_LEFT_RANGE_SENSOR_ADDRESS);
        frontLeftRangeDevice.engage();

        frontCenterRangeDevice = (I2cDeviceSynch)hardwareMap.get("fc");
        frontCenterRangeSensor = new ModernRoboticsI2cRangeSensor(frontCenterRangeDevice);
        frontCenterRangeSensor.setI2cAddress(FRONT_CENTER_RANGE_SENSOR_ADDRESS);
        frontCenterRangeDevice.engage();

        frontRightRangeDevice = (I2cDeviceSynch)hardwareMap.get("fr");
        frontRightRangeSensor = new ModernRoboticsI2cRangeSensor(frontRightRangeDevice);
        frontRightRangeSensor.setI2cAddress(FRONT_RIGHT_RANGE_SENSOR_ADDRESS);
        frontRightRangeDevice.engage();

        rightCenterRangeDevice = (I2cDeviceSynch)hardwareMap.get("rc");
        rightCenterRangeSensor = new ModernRoboticsI2cRangeSensor(rightCenterRangeDevice);
        rightCenterRangeSensor.setI2cAddress(RIGHT_CENTER_RANGE_SENSOR_ADDRESS);
        rightCenterRangeDevice.engage();

        backCenterRangeDevice = (I2cDeviceSynch)hardwareMap.get("bc");
        backCenterRangeSensor = new ModernRoboticsI2cRangeSensor(backCenterRangeDevice);
        backCenterRangeSensor.setI2cAddress(BACK_CENTER_RANGE_SENSOR_ADDRESS);
        backCenterRangeDevice.engage();

        leftCenterRangeDevice = (I2cDeviceSynch)hardwareMap.get("lc");
        leftCenterRangeSensor = new ModernRoboticsI2cRangeSensor(leftCenterRangeDevice);
        leftCenterRangeSensor.setI2cAddress(LEFT_CENTER_RANGE_SENSOR_ADDRESS);
        leftCenterRangeDevice.engage();
    }

    public void loop() {
        telemetry.addData("Front Left Sensor Reads: ", frontLeftRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Center Sensor Reads: ", frontCenterRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Right Sensor Reads: ", frontRightRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Center Sensor Reads: ", rightCenterRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Center Sensor Reads: ", backCenterRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Center Sensor Reads: ", leftCenterRangeSensor.getDistance(DistanceUnit.CM));

        /*
        telemetry.addData("raw ultrasonic", frontCenterRangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", frontCenterRangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", frontCenterRangeSensor.cmOptical());
        telemetry.addData("fc status", frontCenterRangeSensor.status());
        telemetry.addData("lc status", leftCenterRangeSensor.status());
*/
    }

}