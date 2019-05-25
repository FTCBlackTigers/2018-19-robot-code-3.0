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
import org.firstinspires.ftc.teamcode.Util.TurnPIDController;

public class Drive {
    public enum Direction {
        FORWARD, BACKWARD;
    }
    public enum CurvedDirection {
        LEFT, RIGHT;
    }
    public enum Side {
        DEPOT,
        CREATER;
    }

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER_CM = 10.16;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.141592654);
    private final double KP = 1, KI = 0, KD = 0, TOLERANCE = 1;//KP = 0.05, KI = 0.03, KD = 0.03, TOLERANCE = 1;
    private final double turnKP = 0.01, turnKI = 0.0, turnKD = 0.0, turnTOLERANCE = 4;

    private OpMode opMode;
    private LogCreater log;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private BT_Gyro gyro = new BT_Gyro();
    public GoldRecognation recognation = null;

    private PIDController forwardPID;
    private TurnPIDController turnPID;


    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log) {
        this.log = log;
        init(hardwareMap, opMode);
    }
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        gyro.init(hardwareMap);
        /**
        if (opMode instanceof  LinearOpMode) {
            recognation = new GoldRecognation(hardwareMap, opMode);
        }
         **/

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        if(this.opMode instanceof LinearOpMode) {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        forwardPID = new PIDController(KP, KI, KD, TOLERANCE,opMode);
        turnPID = new TurnPIDController(turnKP, turnKI, turnKD, turnTOLERANCE, opMode);

    }
    public void teleOpMotion(Gamepad driver) {
        if (driver.dpad_up) {
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        } else if (driver.dpad_down) {
            leftDrive.setPower(-0.2);
            rightDrive.setPower(-0.2);
        } else tankDrive(-driver.left_stick_y, -driver.right_stick_y);

        //TODO: change telemetry
        opMode.telemetry.addLine("Drive: ").
                addData("left motor power: ", leftDrive.getPower())
                .addData("right motor power: ", rightDrive.getPower())
                .addData("left motor pos: ", leftDrive.getCurrentPosition())
                .addData("right motor pos: ", rightDrive.getCurrentPosition())
                .addData("\ndelta in encoders: ", Math.abs(leftDrive.getCurrentPosition() - rightDrive.getCurrentPosition()));
    }

    private void tankDrive(double powerLeftDrive, double powerRightDrive) {
        leftDrive.setPower(powerLeftDrive * 0.5);
        rightDrive.setPower(powerRightDrive * 0.5);
    }

    public void driveByEncoder(double distanceCm, double speed, Direction direction, double timeS) {
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

            double stopTime = opMode.getRuntime() + timeS;
            while (((LinearOpMode) opMode).opModeIsActive() &&
                    (opMode.getRuntime() < stopTime) &&
                    (rightDrive.isBusy() && leftDrive.isBusy())) {
                opMode.telemetry.addData("leftPos", leftDrive.getCurrentPosition());
                opMode.telemetry.addData("rightPos", rightDrive.getCurrentPosition());
                opMode.telemetry.update();
                if(log != null) {
                    log.writeLog("leftDrive", leftDrive.getCurrentPosition(), "target: " + newLeftTarget);
                    log.writeLog("rightDrive", rightDrive.getCurrentPosition(), "target: " + newRightTarget);
                }
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public  void curvedDrive(double distanceCm, double angleFactor, double speed, Direction direction, CurvedDirection curvedDirection)
    {
        int newLeftTarget;
        int newRightTarget;

        double rightSpeed;
        double leftSpeed;

        if(direction == Direction.BACKWARD) {
            distanceCm *= -1;
        }
        if(curvedDirection == CurvedDirection.RIGHT) {
            newLeftTarget = (int) (distanceCm * COUNTS_PER_CM);
            newRightTarget = (int) (distanceCm * COUNTS_PER_CM / angleFactor);

            rightSpeed = speed / angleFactor;
            leftSpeed = speed;
        }
        else {
            newLeftTarget = (int) (distanceCm * COUNTS_PER_CM  / angleFactor);
            newRightTarget = (int) (distanceCm * COUNTS_PER_CM);

            rightSpeed = speed;
            leftSpeed = speed / angleFactor;
        }

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(leftSpeed));
        rightDrive.setPower(Math.abs(rightSpeed));

        while (((LinearOpMode) opMode).opModeIsActive() &&
                (rightDrive.isBusy() && leftDrive.isBusy())) {
            opMode.telemetry.addData("leftPos", leftDrive.getCurrentPosition());
            opMode.telemetry.addData("rightPos", rightDrive.getCurrentPosition());
            opMode.telemetry.update();
            if(log != null) {
                log.writeLog("leftDrive", leftDrive.getCurrentPosition(), "target: " + newLeftTarget);
                log.writeLog("rightDrive", rightDrive.getCurrentPosition(), "target" + newRightTarget);
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void turnByGyroAbsolut(double targetDegree, double timeS) {
        while (targetDegree > 180 && ((LinearOpMode)opMode).opModeIsActive()) targetDegree -= 360;
        while (targetDegree <= -179 && ((LinearOpMode)opMode).opModeIsActive()) targetDegree += 360;
        turnPID.reset(targetDegree, getAngle());
        timeS += opMode.getRuntime();

        while (!turnPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive() && opMode.getRuntime() <= timeS) {
            while (!turnPID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()&& opMode.getRuntime() <= timeS) {
                double output = turnPID.getOutput(getAngle());
                //TODO: make sure 0.1 is good
                /*if(Math.abs(output) < 0.1) {
                    output = Math.signum(output) * 0.1;
                }*/

                leftDrive.setPower(-output);
                rightDrive.setPower(output);
                opMode.telemetry.addData("error: ", turnPID.getCurrentError())
                        .addData("output: ", output)
                        .addData("robot angle: ", getAngle());
                opMode.telemetry.update();
                if (log != null) {
                    log.writeLog("gyro", getAngle(), "target: " + targetDegree + ", output: " + output);
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
    public void turnByGyroRelative(double degrees, double timeS) {
        turnByGyroAbsolut(this.gyro.getAngle() + degrees, timeS);
    }

    public GoldRecognation.MineralPos sampling(Side side) {
        GoldRecognation.MineralPos pos = recognation.getGoldPos(log);
        recognation.stopTfod();
        recognation.turnOffLeds();
        if (side == Side.DEPOT) {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(40,2);
                    driveByEncoder(10, 0.2, Direction.BACKWARD, 5);
                    curvedDrive(140, 2, 0.5, Direction.BACKWARD, CurvedDirection.LEFT);
                    //driveByEncoder(80, 0.5, Direction.BACKWARD, 5);
                    //turnByGyroAbsolut(-30,2);
                    //driveByEncoder(45,0.5,  Direction.BACKWARD, 5);
                    break;
                case CENTER:
                case UNKNOWN:
                    turnByGyroAbsolut(5,3);
                    driveByEncoder(75,0.5, Direction.BACKWARD, 10);
                    break;
                case RIGHT:
                    turnByGyroAbsolut(-40,10);
                    //driveByEncoder(10, 0.2, Direction.BACKWARD, 5);
                    curvedDrive(140, 2, 0.5, Direction.BACKWARD, CurvedDirection.RIGHT);
                    //driveByEncoder(90,0.5, Direction.BACKWARD,10);
                    //turnByGyroAbsolut(30,10);
                    //driveByEncoder(45,0.5, Direction.BACKWARD,10);
                    break;
            }

        } else {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(30,10);
                    driveByEncoder(45,0.5, Direction.BACKWARD,10);
                    break;
                case CENTER:
                case UNKNOWN:
                    turnByGyroAbsolut(5,10);
                    driveByEncoder(40,0.5, Direction.BACKWARD,10);
                    break;
                case RIGHT:
                    turnByGyroAbsolut(-30,10);
                    driveByEncoder(45,0.5, Direction.BACKWARD,10);
                    break;
            }
        }
        opMode.telemetry.addData("mineral", pos);
        return pos;
    }

    public void samplingCam(Side side, GoldRecognation.MineralPos pos){
        if (side == Side.DEPOT) {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(40,2);
                    driveByEncoder(10, 0.2, Direction.BACKWARD, 5);
                    curvedDrive(140, 2, 0.5, Direction.BACKWARD, CurvedDirection.LEFT);
                    break;
                case CENTER:
                case UNKNOWN:
                    turnByGyroAbsolut(5,3);
                    driveByEncoder(75,0.5, Direction.BACKWARD, 10);
                    break;
                case RIGHT:
                    turnByGyroAbsolut(-30,10);
                    driveByEncoder(50, 0.5, Drive.Direction.BACKWARD, 5);
                    driveByEncoder(30, 0.5, Drive.Direction.FORWARD, 5);
                    turnByGyroAbsolut(-100, 5);
                    driveByEncoder(80, 0.5, Drive.Direction.FORWARD, 5);
                    turnByGyroAbsolut(-30 , 5);
                    driveByEncoder(80, 0.5, Drive.Direction.BACKWARD, 5);
                    //curvedDrive(140, 2, 0.5, Direction.BACKWARD, CurvedDirection.RIGHT);
                    break;
            }
            } else {
            switch (pos) {
                case LEFT:
                    turnByGyroAbsolut(-135,10);
                    driveByEncoder(45,0.5, Direction.FORWARD,10);
                    break;
                case CENTER:
                case UNKNOWN:
                    turnByGyroAbsolut(185,10);
                    driveByEncoder(40,0.5, Direction.FORWARD,10);
                    break;
                case RIGHT:
                    turnByGyroRelative(140,5);
                    driveByEncoder(45,0.5, Direction.FORWARD,10);
                    break;
            }
        }
        opMode.telemetry.addData("mineral", pos);
    }


    public double getAngle(){
        return gyro.getAngle();
    }

    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
    }

    public void setLog(LogCreater log) {
        this.log = log;
    }


     public void driveByEncoderUsingPID(double distance,Direction direction) {
         if (direction == Direction.BACKWARD) {
             distance *= -1;
         }
         leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         forwardPID.reset(distance, 0);

         opMode.telemetry.addData("error: ", forwardPID.getCurrentError());
         opMode.telemetry.update();
         while (!forwardPID.onTarget() && ((LinearOpMode) opMode).opModeIsActive()) {
             while (!forwardPID.onTarget() && ((LinearOpMode) opMode).opModeIsActive()) {
                 double output = forwardPID.getOutput(leftDrive.getCurrentPosition() / COUNTS_PER_CM);
                 leftDrive.setPower(output);
                 rightDrive.setPower(output);
                 opMode.telemetry.addData("error: ", forwardPID.getCurrentError())
                         .addData("output: ", output);
                 opMode.telemetry.update();
                 if (log != null) {
                     log.writeLog("leftDrive", leftDrive.getCurrentPosition() / COUNTS_PER_CM, "target: " + distance);
                     log.writeLog("rightDrive", rightDrive.getCurrentPosition() / COUNTS_PER_CM, "target" + distance);
                 }
             }
             double time = opMode.getRuntime() + 0.3;
            /// while ((opMode.getRuntime() < time) && ((LinearOpMode)opMode).opModeIsActive()) {
                  //forwardPID.updateError(getAngle());
            // }
         }
         leftDrive.setPower(0);
         rightDrive.setPower(0);
     }

}
