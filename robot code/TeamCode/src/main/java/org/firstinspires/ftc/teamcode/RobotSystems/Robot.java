package org.firstinspires.ftc.teamcode.RobotSystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.LogCreater;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.Date;

public class Robot {

    public boolean initilaized = false;
    public Drive drive = new Drive();
    public Intake intake = new Intake();
    public Climbing climbing = new Climbing();
    private LogCreater log;
    private OpMode opMode;


    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log) {
        this.log = log;
        drive.init(hardwareMap, opMode, log);
        intake.init(hardwareMap, opMode, log);
        climbing.init(hardwareMap, opMode, log);
        this.opMode = opMode;
        initilaized = true;
        opMode.telemetry.addLine("robot initialized");
        opMode.telemetry.update();
    }


    public void init(HardwareMap hardwareMap, OpMode opMode) {
        drive.init(hardwareMap, opMode);
        intake.init(hardwareMap, opMode);
        climbing.init(hardwareMap, opMode);
        this.opMode = opMode;

        opMode.telemetry.addLine("robot initialized");
        opMode.telemetry.update();

    }
    public void teleop(Gamepad driver, Gamepad operator, boolean isTRX) {
        if (isTRX) {
            drive.trxDrive(driver);
        } else {
            drive.teleOpMotion(driver);

        }
        intake.teleOpMotion(driver, operator);
        climbing.teleOpMotion(operator);

    }

    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
        this.climbing.setOpMode(opMode);
        this.drive.setOpMode(opMode);
        this.intake.setOpMode(opMode);
    }

    public void setLog(LogCreater log) {
        this.log = log;
        this.climbing.setLog(log);
        this.drive.setLog(log);
        this.intake.setLog(log);
    }
}
