package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Pixy.PixyBlock;
import org.firstinspires.ftc.teamcode.Pixy.PixyCam;
import org.firstinspires.ftc.teamcode.Util.LogCreater;
import org.firstinspires.ftc.teamcode.Util.PIDController;
import org.firstinspires.ftc.teamcode.Util.GoldRecognation;

public class Drive {
    public enum Direction {
        FORWARD, BACKWARD;
    }
    public enum Side {
        DEPOT,
        CREATER;
    }

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER_CM = 10.16;
    static final double COUNTS_PER_CM = 20; /*(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.141592654);*/
    private final double KP = 0.05, KI = 0.03, KD = 0.03, TOLERANCE = 1;
    private final double turnKP = 0.01, turnKI = 0.00009, turnKD = 0.002, turnTOLERANCE = 5;

    private OpMode opMode;
    private BT_Gyro gyro = new BT_Gyro();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private GoldRecognation recognation = null;
    private PIDController forwardPID;
    private PIDController turnPID;
    private LogCreater log;

    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log) {
        this.log = log;
        init(hardwareMap, opMode);
    }
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        gyro.init(hardwareMap);
        if (opMode instanceof  LinearOpMode) {
            recognation = new GoldRecognation(hardwareMap, opMode);
        }

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        forwardPID = new PIDController(KP, KI, KD, TOLERANCE,opMode);
        turnPID = new PIDController(turnKP, turnKI, turnKD, turnTOLERANCE, opMode);
    }
    public void teleOpMotion(Gamepad driver) {
        if (driver.dpad_up) {
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        } else if (driver.dpad_down) {
            leftDrive.setPower(-0.2);
            rightDrive.setPower(-0.2);
        } else tankDrive(-driver.left_stick_y, -driver.right_stick_y);

        opMode.telemetry.addLine("Drive: ").
                addData("left motor power: ", leftDrive.getPower()).
                addData("right motor power: ", rightDrive.getPower())
                .addData("left motor pos: ", leftDrive.getCurrentPosition())
                .addData("right motor pos: ", rightDrive.getCurrentPosition());
    }


    private void tankDrive(double powerLeftDrive, double powerRightDrive) {
        leftDrive.setPower(powerLeftDrive * 0.5);
        rightDrive.setPower(powerRightDrive * 0.5);
    }

    public void turnByGyroRelative(double degrees) {
        turnByGyroAbsolut(this.gyro.getAngle() + degrees);
    }

    public void trxDrive(Gamepad driver) {
        if (driver.a && driver.right_trigger != 0) {
            leftDrive.setPower(driver.right_trigger);
            rightDrive.setPower(-driver.right_trigger);
        } else if (driver.a && driver.left_trigger != 0) {
            leftDrive.setPower(-driver.left_trigger);
            rightDrive.setPower(driver.left_trigger);
        } else if (driver.right_trigger != 0) {
            leftDrive.setPower(driver.right_trigger);
            rightDrive.setPower(driver.right_trigger);
        } else if (driver.left_trigger != 0) {
            leftDrive.setPower(-driver.left_trigger);
            rightDrive.setPower(-driver.left_trigger);
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }


    public void driveByEncoder(double distanceCm, double speed, Direction direction, double timeMs) {
        if (opMode instanceof LinearOpMode) {
            int newLeftTarget = 0;
            int newRightTarget = 0;

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (direction == Direction.FORWARD) {
                newLeftTarget = (int) (distanceCm * COUNTS_PER_CM);
                newRightTarget = (int) (distanceCm * COUNTS_PER_CM);
            } else if (direction == Direction.BACKWARD) {
                newLeftTarget = (int) (distanceCm * COUNTS_PER_CM * -1);
                newRightTarget = (int) (distanceCm * COUNTS_PER_CM * -1);
            }

            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            double stopTime = opMode.getRuntime() + timeMs;
            while (((LinearOpMode) opMode).opModeIsActive() &&
                    (opMode.getRuntime() < stopTime) &&
                    (rightDrive.isBusy() && leftDrive.isBusy())) {
                opMode.telemetry.addData("leftPos", leftDrive.getCurrentPosition());
                opMode.telemetry.addData("rightPos", rightDrive.getCurrentPosition());
                opMode.telemetry.update();
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveByEncoderUsingPID(double distance,Direction direction){
        if(direction == Direction.BACKWARD){
            distance *= -1;
        }
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forwardPID.reset(distance,0);

        opMode.telemetry.addData("error: " , forwardPID.getCurrentError());
        opMode.telemetry.update();
        while (!forwardPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()){
            double output = forwardPID.getOutput(leftDrive.getCurrentPosition() / COUNTS_PER_CM);
            leftDrive.setPower(output);
            rightDrive.setPower(output);
            opMode.telemetry.addData("error: " , forwardPID.getCurrentError())
            .addData("output: " , output);
            opMode.telemetry.update();
            if (log != null) {
                log.writeLog("leftDrive", leftDrive.getCurrentPosition() / COUNTS_PER_CM, "target: " + distance);
                log.writeLog("rightDrive", rightDrive.getCurrentPosition() / COUNTS_PER_CM, "target" + distance);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnByGyroAbsolut(double targetDegree) {
        while (targetDegree > 180) targetDegree -= 360;
        while (targetDegree <= -180) targetDegree += 360;
        turnPID.reset(targetDegree, getAngle());
        while (!turnPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()) {
            while (!turnPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()) {
                double output = turnPID.getOutput(getAngle());
                leftDrive.setPower(-output);
                rightDrive.setPower(output);
                opMode.telemetry.addData("error: ", turnPID.getCurrentError())
                        .addData("output: ", output)
                        .addData("robot angle: ", getAngle());
                opMode.telemetry.update();
                if (log != null) {
                    log.writeLog("gyro", getAngle(), "target" + targetDegree);
                }
            }
            double time = opMode.getRuntime() + 0.3;
            while ((opMode.getRuntime() < time) && ((LinearOpMode)opMode).opModeIsActive()) {
                turnPID.updateError(getAngle());
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void Sampling(Side side) {
        GoldRecognation.MineralPos pos = recognation.getGoldPos(log);
        recognation.stopTfod();
        recognation.turnOffLeds();
        if (side == Side.DEPOT) {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(30);
                    driveByEncoderUsingPID(100, Direction.BACKWARD);
                    turnByGyroAbsolut(-30);
                    driveByEncoderUsingPID(50,Direction.BACKWARD);
                    break;
                case CENTER:
                case UNKNOWN:
                    turnByGyroAbsolut(5);
                    driveByEncoderUsingPID(130, Direction.BACKWARD);
                    break;
                case RIGHT:
                    turnByGyroAbsolut(-30);
                    driveByEncoderUsingPID(85, Direction.BACKWARD);
                    turnByGyroAbsolut(30);
                    driveByEncoderUsingPID(50,Direction.BACKWARD);
                    break;
            }

        } else {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(30);
                    driveByEncoderUsingPID(75, Direction.BACKWARD);
                    turnByGyroAbsolut(0);
                    driveByEncoderUsingPID(20, Drive.Direction.BACKWARD);

                    break;
                case CENTER:
                case UNKNOWN:
                    turnByGyroAbsolut(5);
                    driveByEncoderUsingPID(60, Direction.BACKWARD);
                    break;
                case RIGHT:
                    turnByGyroAbsolut(-30);
                    driveByEncoderUsingPID(75, Direction.BACKWARD);
                    turnByGyroAbsolut(0);
                    driveByEncoderUsingPID(20, Drive.Direction.BACKWARD);
                    break;
            }
        }
        opMode.telemetry.addData("mineral", pos);
        }

        public void driveToCreater(Side side) {
        GoldRecognation.MineralPos pos = recognation.getGoldPos(log);
        if (side == Side.DEPOT) {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(90);
                    driveByEncoderUsingPID(80, Direction.BACKWARD);
                    turnByGyroAbsolut(20);
                    driveByEncoderUsingPID(60, Direction.BACKWARD);
                    break;
                case UNKNOWN:
                case CENTER:
                    turnByGyroAbsolut(90);
                    driveByEncoderUsingPID(60, Direction.BACKWARD);
                    turnByGyroAbsolut(20);
                    driveByEncoderUsingPID(60, Direction.BACKWARD);
                    break;
                case RIGHT:
                    turnByGyroAbsolut(20);
                    turnByGyroAbsolut(110);

            }
        }
        }

    public double getAngle(){
        return gyro.getAngle();
    }


}




