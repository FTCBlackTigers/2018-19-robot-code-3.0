package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.GlobalVariebels;
import org.firstinspires.ftc.teamcode.Util.LogCreater;
import org.firstinspires.ftc.teamcode.Util.PIDController;

import static java.lang.Thread.sleep;

public class Climbing {
    public enum Angle {
        DOWN(25),
        DRIVE_POS(72),
        COLLECT(170),
        LAND(68),
        GO_TO_CLIMB(60),
        CLIMB(30),
        AUTO_PUT(45),
        PUT(67);
        float pos;
        final double ticksPerDegree = 93.588;

        public int getTicks() {
            return ((int) (ticksPerDegree * pos));
        }

        Angle(float ang) {
            this.pos = ang;
        }
    }

    public enum Height {
        DRIVE_POS(0),
        COLLECT(15),
        LAND(25),
        GO_TO_CLIMB(22),
        CLIMB(10),
        PUT(30);

        float pos;
        final double ticksPerCm = 50.63;

        public int getTicks() {
            return ((int) (ticksPerCm * pos));
        }

        Height(float pos) {
            this.pos = pos;
        }
    }

    private final double LIFT_SPEED = 0.7;
    private final double ANGLE_SPEED = 1;

    private final double HANG_OPEN_POS = 0.85;
    private final double HANG_CLOSE_POS = 0;

    private final double KP = 0.5, KI = 0, KD = 0, TOLERANCE = 4;

    private OpMode opMode;
    private LogCreater log;

    public DcMotor angleMotorLeft;
    public DcMotor angleMotorRight;
    public DcMotor liftMotorLeft;
    public DcMotor liftMotorRight;
    private Servo hangServo;
    private DigitalChannel liftTouch;
    private DigitalChannel angleTouch;
    private AnalogInput potentiometer;

    private boolean angleTouchIsPrevActive;
    private boolean angleTouchIsActive;
    private boolean liftTouchIsPrevActive;
    private boolean liftTouchIsActive;

    private double angleJoystickValue = 0;
    private double angleJoystickPrevValue = 0;
    private double liftJoystickValue = 0;
    private double liftJoystickPrevValue = 0;
    private double stopPIDTime;

    private PIDController anglePID;

    public void init(HardwareMap hardwareMap, OpMode opMode, LogCreater log) {
        this.log = log;
        init(hardwareMap, opMode);
    }
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class,"liftMotorRight");
        angleMotorLeft = hardwareMap.get(DcMotor.class, "angleMotorLeft");
        angleMotorRight = hardwareMap.get(DcMotor.class, "angleMotorRight");
        hangServo = hardwareMap.get(Servo.class, "hangServo");
        angleTouch = hardwareMap.get(DigitalChannel.class, "angle_touch");
        liftTouch = hardwareMap.get(DigitalChannel.class, "lift_touch");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        anglePID = new PIDController(KP, KI, KD, TOLERANCE, opMode);

        angleMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);
        angleMotorLeft.setPower(0);
        angleMotorRight.setPower(0);

        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        angleMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        angleMotorRight.setDirection(DcMotor.Direction.REVERSE);

        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftTouch.setMode(DigitalChannel.Mode.INPUT);
        angleTouch.setMode(DigitalChannel.Mode.INPUT);

        hangServo.setDirection(Servo.Direction.REVERSE);

        lockServo();
    }

    public void teleOpMotion(Gamepad operator) {
        angleTouchIsActive = !angleTouch.getState();
        liftTouchIsActive = !liftTouch.getState();

        angleJoystickValue = -operator.left_stick_y;
        liftJoystickValue = -operator.right_stick_y;

        if (angleTouchIsActive && !angleTouchIsPrevActive) {
            angleMotorLeft.setPower(0);
            angleMotorRight.setPower(0);
        }

        if (liftTouchIsActive && !liftTouchIsPrevActive) {
            liftMotorLeft.setPower(0);
            liftMotorRight.setPower(0);
            liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            GlobalVariebels.liftPosEndAuto = 0;
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "reset encoder");
                log.writeLog("liftMotorRight",liftMotorRight.getCurrentPosition(),"reset encoder");
            }
        }

        if ((!liftMotorLeft.isBusy() && liftMotorLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION) || (!liftMotorRight.isBusy() && liftMotorRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION)) {
            //liftMotorLeft.setPower(0);
            //liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //liftMotorRight.setPower(0);
            //liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "arrived to target " + liftMotorLeft.getTargetPosition());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "arrived to target " + liftMotorRight.getTargetPosition());

            }
        } else if((liftMotorLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION) && (liftMotorRight.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ) {
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "target: " + liftMotorLeft.getTargetPosition());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "target: " + liftMotorRight.getTargetPosition());
            }
        }

        if (anglePID.isRunning()) {
            if (anglePID.onTarget()) {
                angleMotorRight.setPower(0);
                angleMotorLeft.setPower(0);
                if (opMode.getRuntime() >= stopPIDTime) {
                    //anglePID.stop();
                    if (log != null) {
                        log.writeLog("angleMotor", getAngle(), "arrived to target " + anglePID.getSetpoint());
                    }
                }
                anglePID.updateError(getAngle());
            } else {
                stopPIDTime = opMode.getRuntime() + 0.3;
                double output = anglePID.getOutput(getAngle());
                if(IsLiftInDeadZone()){
                    output *= 0.4;
                    opMode.telemetry.addData("output: " , output);
                }
                angleMotorLeft.setPower(output);
                angleMotorRight.setPower(output);
                if (log != null) {
                    log.writeLog("angleMotor", getAngle(), "target: " + anglePID.getSetpoint());
                }
            }
        }

        if (operator.dpad_down) {
            moveLift(Height.DRIVE_POS);
            moveAngle(Angle.DRIVE_POS);
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "command to move to drive pos " + Height.DRIVE_POS.getTicks());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "command to move to drive pos " + Height.DRIVE_POS.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to drive pos " + Angle.DRIVE_POS.pos);
            }
        }

        if (operator.dpad_left) {
            moveLift(Height.CLIMB);
            moveAngle(Angle.CLIMB);
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "command to move to climb " + Height.CLIMB.getTicks());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "command to move to climb " + Height.CLIMB.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to climb " + Angle.CLIMB.pos);
            }
        }

        if (operator.dpad_up) {
            moveLift(Height.GO_TO_CLIMB);
            moveAngle(Angle.GO_TO_CLIMB);
            openServo();
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "command to move to go to climb " + Height.GO_TO_CLIMB.getTicks());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "command to move to climb " + Height.CLIMB.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to go to climb " + Angle.GO_TO_CLIMB.pos);
            }
        }

        if (operator.right_trigger > 0.7) {
           moveLift(Height.COLLECT);
           moveAngle(Angle.COLLECT);
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "command to move to climb " + Height.COLLECT.getTicks());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "command to move to climb " + Height.CLIMB.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to collect " + Angle.COLLECT.pos);
            }
        }

        if (operator.left_trigger > 0.7) {
            moveLift(Height.PUT);
            moveAngle(Angle.PUT);
            if (log != null) {
                log.writeLog("liftMotorLeft", liftMotorLeft.getCurrentPosition(), "command to move to put " + Height.PUT.getTicks());
                log.writeLog("liftMotorRight", liftMotorRight.getCurrentPosition(), "command to move to put " + Height.CLIMB.getTicks());
                log.writeLog("angleMotor", getAngle(), "command to move to put " + Angle.PUT.pos);
            }
        }

        if (Math.abs(liftJoystickValue) > 0.1) {
            liftMoveManual(liftJoystickValue);
        } else if (Math.abs(liftJoystickPrevValue) > 0.1) {
            liftMoveManual(0);
        }

        if (Math.abs(angleJoystickValue) > 0.1) {
            angleMoveManual(angleJoystickValue);
        } else if (Math.abs(angleJoystickPrevValue) > 0.1) {
            angleMoveManual(0);
        }

        if (operator.a) {
            openServo();
        }

        if (operator.x) {
            lockServo();
        }


        opMode.telemetry.addLine("climbing: \n")
                .addData("left lift Position: ", liftMotorLeft.getCurrentPosition() + "\n")
                .addData("right lift position: ", liftMotorRight.getCurrentPosition() +"\n")
                .addData("Servo position: ", hangServo.getPosition() + "\n")
                .addData(" potentiometer: ", potentiometer.getVoltage())
                .addData("angle: ", getAngle())
                .addData("liftTouch: " , liftTouchIsActive)
                .addData(" angleTouch: ", angleTouchIsActive +"\n")
                .addData("liftPosEndAuto", GlobalVariebels.liftPosEndAuto);


        angleTouchIsPrevActive = angleTouchIsActive;
        liftTouchIsPrevActive = liftTouchIsActive;

        angleJoystickPrevValue = angleJoystickValue;
        liftJoystickPrevValue = liftJoystickValue;
    }

    public void moveLift(Height height) {
        int targetPosition = height.getTicks() - ((opMode instanceof LinearOpMode) ?  0 : GlobalVariebels.liftPosEndAuto);
        liftMotorLeft.setTargetPosition(targetPosition);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setTargetPosition(targetPosition);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorLeft.setPower(Math.abs(LIFT_SPEED));
        liftMotorRight.setPower(Math.abs(LIFT_SPEED));
    }

    public void moveAngle(Angle angle) {
        anglePID.reset(angle.pos, getAngle());
        double output = anglePID.getOutput(getAngle());
        if(IsLiftInDeadZone()){
            output *= 0.4;
            opMode.telemetry.addData("output: " , output);
        }
        angleMotorLeft.setPower(output);
        angleMotorRight.setPower(output);
    }

    public void liftMoveManual(double motorPower) {
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (liftTouchIsActive && motorPower < 0) {
            return;
        }
        liftMotorLeft.setPower(motorPower * LIFT_SPEED);
        liftMotorRight.setPower(motorPower * LIFT_SPEED);
    }

    public void angleMoveManual(double motorPower) {
        anglePID.stop();

        if(angleTouchIsActive && motorPower < 0) {
            return;
        }
        angleMotorLeft.setPower(motorPower * ANGLE_SPEED);
        angleMotorRight.setPower(motorPower * ANGLE_SPEED);
    }

    public void lockServo() {
        hangServo.setPosition(HANG_CLOSE_POS);
    }

    public void openServo() {
        hangServo.setPosition(HANG_OPEN_POS);
    }


    //Auto methods
    public void moveLiftAuto(Height height, double timeS) {
        moveLift(height);
        double timeToStop = timeS + opMode.getRuntime();
        while(liftMotorLeft.isBusy() && liftMotorRight.isBusy() && ((LinearOpMode) opMode).opModeIsActive() && opMode.getRuntime() <= timeToStop) {
            opMode.telemetry.addData("liftMotorLeft: ", liftMotorLeft.getCurrentPosition());
            opMode.telemetry.update();
            if (log != null) {
                log.writeLog("lift", liftMotorLeft.getCurrentPosition(), "target: " + height.getTicks());
            }
        }
        liftMotorLeft.setPower(0);
    }

    public void moveLiftAuto(Height height){
        moveLiftAuto(height, 3);
    }

    public void moveAngleAuto(Angle angle) {
        anglePID.reset(angle.pos, getAngle());
        while (!anglePID.onTarget() && ((LinearOpMode)opMode).opModeIsActive()) {
            double output = anglePID.getOutput(getAngle());
            angleMotorRight.setPower(output);
            angleMotorLeft.setPower(output);
            opMode.telemetry.addData("error: ", anglePID.getCurrentError())
                    .addData("output: ", output)
                    .addData("angle", getAngle());
            opMode.telemetry.update();
            if (log != null) {
                log.writeLog("angle", getAngle(), "target: " + angle.pos);
            }
        }
        angleMotorRight.setPower(0);
        angleMotorLeft.setPower(0);
    }

    public void moveAngleAndHeight(Angle angle, Height height){
        anglePID.reset(angle.pos, getAngle());

        liftMotorLeft.setTargetPosition(height.getTicks());
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setTargetPosition(height.getTicks());
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setPower(Math.abs(LIFT_SPEED));
        liftMotorLeft.setPower(Math.abs(LIFT_SPEED));

        while (!anglePID.onTarget()  && ((LinearOpMode) opMode).opModeIsActive()){
            double output = anglePID.getOutput(getAngle());
            angleMotorRight.setPower(output);
            angleMotorLeft.setPower(output);
            opMode.telemetry.addLine("Angle \n")
                    .addData("error: ", anglePID.getCurrentError())
                    .addData("output: ", output)
                    .addData("angle", getAngle() + "\n");

            opMode.telemetry.addLine("Lift \n")
                    .addData("position: ", liftMotorLeft.getCurrentPosition());
            opMode.telemetry.update();

            if(!liftMotorLeft.isBusy() || !liftMotorRight.isBusy()){
                liftMotorLeft.setPower(0);
                liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotorRight.setPower(0);
                liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (log != null) {
                log.writeLog("angle", getAngle(), "target: " + angle.pos);
                log.writeLog("lift", liftMotorLeft.getCurrentPosition(), "target: " + height.getTicks());
            }

        }
        angleMotorLeft.setPower(0);
        angleMotorRight.setPower(0);
        double timeToStop = opMode.getRuntime() + 2;
        while(liftMotorLeft.isBusy()  && liftMotorRight.isBusy() && ((LinearOpMode) opMode).opModeIsActive() && opMode.getRuntime() <= timeToStop){
            opMode.telemetry.addLine("Lift \n")
                    .addData("position: ", liftMotorLeft.getCurrentPosition());
            opMode.telemetry.update();
            if (log != null) {
                log.writeLog("lift", liftMotorLeft.getCurrentPosition(), "target: " + height.getTicks());
            }
        }

        liftMotorLeft.setPower(0);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setPower(0);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public double getAngle() {
        return potentiometer.getVoltage() / 3.347 *270;
    }

    public void setOpMode(OpMode opMode) {
        this.opMode = opMode;
    }

    public void setLog(LogCreater log) {
        this.log = log;
    }
    public boolean IsLiftInDeadZone(){
        return getAngle()>49 && getAngle()<80;
    }
        /* TESTING CODE
    public void moveAngleByPID(double target) {
        anglePID.reset(target, getAngle());
        while (!anglePID.onTarget()  && ((LinearOpMode) opMode).opModeIsActive()) {
            double output = anglePID.getOutput(getAngle());
            angleMotorRight.setPower(output);
            angleMotorLeft.setPower(output);
            opMode.telemetry.addData("error: " ,anglePID.getCurrentError())
                    .addData("output: " , output)
                    .addData("angle", getAngle());
            opMode.telemetry.update();
        }
        angleMotorRight.setPower(0);
        angleMotorLeft.setPower(0);
        while (((LinearOpMode) opMode).opModeIsActive()) {
            opMode.telemetry.addData("error: " ,anglePID.getCurrentError())
                    .addData("angle", getAngle());
            opMode.telemetry.update();
        }
    }
    */
}
