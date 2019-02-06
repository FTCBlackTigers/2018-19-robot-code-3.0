package org.firstinspires.ftc.teamcode.Pixy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 7/3/2017.
 */

@TeleOp(name="Test Pixy Cam", group = "Pixy Cam")
@Disabled
public class pixyTest extends LinearOpMode {
    I2cDevice pixyCam;
    I2cDeviceSynchImpl pixyCamReader;
    I2cAddr pixyCamAddress = I2cAddr.create8bit(0x01);
    double x, y, width, height, numObjs;
    byte[] pixyData;

    @Override
    public void runOpMode() throws InterruptedException {
        pixyCam = hardwareMap.i2cDevice.get("pixy");
        pixyCamReader = new I2cDeviceSynchImpl(pixyCam, pixyCamAddress, false);
        pixyCamReader.engage();

        waitForStart();

        while(opModeIsActive()){
            pixyCamReader.engage();
            pixyData = pixyCamReader.read(0x51, 5);

            x = pixyData[1];
            y = pixyData[2];
            numObjs = pixyData[0];

            telemetry.addData("0", pixyData[0]);
            telemetry.addData("1", pixyData[1]);
            telemetry.addData("2", pixyData[2]);
            telemetry.addData("3", pixyData[3]);
            telemetry.addData("4", pixyData[4]);
            telemetry.addData("Length", pixyData.length);
            telemetry.update();
            pixyCam.readI2cCacheFromController();
        }

    }
}
