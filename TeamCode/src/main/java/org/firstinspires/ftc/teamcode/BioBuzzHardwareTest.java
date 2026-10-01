/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Hardware test OpMode for the BioBuzz StarterBot.
 *
 * Only ONE device runs at a time, so you can test each piece of hardware in isolation.
 * Every other device is forced to zero power every loop.
 *
 * CONTROLS (gamepad1)
 *   D-pad Up / Down    : select previous / next device
 *   D-pad Left / Right : decrease / increase the power limit (10% steps)
 *   Right Trigger      : run selected device forward  (scaled by power limit)
 *   Left Trigger       : run selected device reverse  (scaled by power limit)
 *   B                  : EMERGENCY STOP (zero everything, power limit back to 50%)
 *   Y                  : reset encoder for selected motor
 *   X                  : (Launcher only) toggle POWER mode / VELOCITY mode
 *   A                  : (Launcher, VELOCITY mode only) hold to spin to target velocity
 *   Left / Right Bumper: (Launcher, VELOCITY mode only) target velocity -/+ 50 ticks/sec
 *
 * Start with the power limit low (default 30%) the first time you run anything.
 */

@TeleOp(name = "BioBuzz Hardware Test", group = "StarterBot")
//@Disabled
public class BioBuzzHardwareTest extends OpMode {

    // ---------- Devices ----------
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx intake = null;
    private DcMotorEx launcher = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // ---------- Test menu ----------
    private static final String[] DEVICE_NAMES = {
            "Left Drive Motor",
            "Right Drive Motor",
            "Intake Motor",
            "Launcher Motor",
            "Left Intake Servo",
            "Right Intake Servo",
            "Windmill Servo"
    };
    private static final int LEFT_DRIVE = 0;
    private static final int RIGHT_DRIVE = 1;
    private static final int INTAKE = 2;
    private static final int LAUNCHER = 3;
    private static final int LEFT_INTAKE_SERVO = 4;
    private static final int RIGHT_INTAKE_SERVO = 5;
    private static final int WINDMILL_SERVO = 6;

    private int selected = 0;

    // ---------- Settings ----------
    private double powerLimit = 0.3;
    private boolean launcherVelocityMode = false;
    private double launcherTargetVelocity = 1250; // ticks/sec (matches teleop)
    private static final double VELOCITY_STEP = 50;
    private static final double MAX_LAUNCHER_VELOCITY = 2800; // ~6000 RPM @ 28 ticks/rev

    // ---------- Edge detection ----------
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean prevX, prevY, prevLB, prevRB;

    // ---------- Telemetry values ----------
    private double commandedPower = 0;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        // Same directions as the main teleop so "forward" means the same thing here.
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Reset encoders, then run all motors open-loop (raw power) for testing.
        for (DcMotorEx m : new DcMotorEx[]{leftDrive, rightDrive, intake, launcher}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // PIDF is only used when the launcher is in VELOCITY mode.
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        stopAll();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "DPad U/D = select | DPad L/R = power limit");
        telemetry.addData("", "RT = fwd, LT = rev | B = E-STOP | Y = reset encoder");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        stopAll();
    }

    @Override
    public void loop() {
        handleMenuInput();

        // E-STOP
        if (gamepad1.b) {
            stopAll();
            powerLimit = 0.5;
            telemetry.addData("!!! E-STOP !!!", "Released B to continue");
            telemetry.update();
            return;
        }

        // Reset encoder on the selected motor (edge-triggered)
        if (gamepad1.y && !prevY) {
            resetSelectedEncoder();
        }
        prevY = gamepad1.y;

        // Launcher mode toggle (edge-triggered)
        if (selected == LAUNCHER && gamepad1.x && !prevX) {
            toggleLauncherMode();
        }
        prevX = gamepad1.x;

        // Launcher target velocity adjust (edge-triggered)
        if (selected == LAUNCHER && launcherVelocityMode) {
            if (gamepad1.right_bumper && !prevRB) {
                launcherTargetVelocity = Math.min(MAX_LAUNCHER_VELOCITY, launcherTargetVelocity + VELOCITY_STEP);
            }
            if (gamepad1.left_bumper && !prevLB) {
                launcherTargetVelocity = Math.max(0, launcherTargetVelocity - VELOCITY_STEP);
            }
        }
        prevRB = gamepad1.right_bumper;
        prevLB = gamepad1.left_bumper;

        // Trigger-based power command, scaled by the limit
        commandedPower = (gamepad1.right_trigger - gamepad1.left_trigger) * powerLimit;

        runSelectedDevice();
        updateTelemetry();
    }

    @Override
    public void stop() {
        stopAll();
    }

    // =====================================================================
    // Input handling
    // =====================================================================

    private void handleMenuInput() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !prevUp) {
            selected = (selected - 1 + DEVICE_NAMES.length) % DEVICE_NAMES.length;
            stopAll(); // never carry power over when switching devices
        }
        if (down && !prevDown) {
            selected = (selected + 1) % DEVICE_NAMES.length;
            stopAll();
        }
        if (right && !prevRight) {
            powerLimit = Math.min(1.0, Math.round((powerLimit + 0.1) * 10) / 10.0);
        }
        if (left && !prevLeft) {
            powerLimit = Math.max(0.1, Math.round((powerLimit - 0.1) * 10) / 10.0);
        }

        prevUp = up;
        prevDown = down;
        prevLeft = left;
        prevRight = right;
    }

    // =====================================================================
    // Hardware control
    // =====================================================================

    /** Drives ONLY the selected device; every other device is held at zero. */
    private void runSelectedDevice() {
        double p = commandedPower;

        leftDrive.setPower(selected == LEFT_DRIVE ? p : 0);
        rightDrive.setPower(selected == RIGHT_DRIVE ? p : 0);
        intake.setPower(selected == INTAKE ? p : 0);
        leftIntakeServo.setPower(selected == LEFT_INTAKE_SERVO ? p : 0);
        rightIntakeServo.setPower(selected == RIGHT_INTAKE_SERVO ? p : 0);
        windmillServo.setPower(selected == WINDMILL_SERVO ? p : 0);

        if (selected == LAUNCHER) {
            if (launcherVelocityMode) {
                launcher.setVelocity(gamepad1.a ? launcherTargetVelocity : 0);
            } else {
                launcher.setPower(p);
            }
        } else {
            if (launcherVelocityMode) {
                launcher.setVelocity(0);
            } else {
                launcher.setPower(0);
            }
        }
    }

    private void toggleLauncherMode() {
        launcher.setPower(0);
        launcherVelocityMode = !launcherVelocityMode;
        if (launcherVelocityMode) {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
        } else {
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void resetSelectedEncoder() {
        DcMotorEx m = getSelectedMotor();
        if (m == null) return;

        DcMotor.RunMode previous = m.getMode();
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(previous);
    }

    private DcMotorEx getSelectedMotor() {
        switch (selected) {
            case LEFT_DRIVE:
                return leftDrive;
            case RIGHT_DRIVE:
                return rightDrive;
            case INTAKE:
                return intake;
            case LAUNCHER:
                return launcher;
            default:
                return null; // servos have no encoder
        }
    }

    private void stopAll() {
        commandedPower = 0;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intake.setPower(0);
        if (launcherVelocityMode) {
            launcher.setVelocity(0);
        } else {
            launcher.setPower(0);
        }
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }

    // =====================================================================
    // Telemetry
    // =====================================================================

    private void updateTelemetry() {
        telemetry.addLine("=== BioBuzz Hardware Test ===");
        telemetry.addData("Selected", "%d/%d: %s", selected + 1, DEVICE_NAMES.length, DEVICE_NAMES[selected]);
        telemetry.addData("Power Limit", "%.0f%%  (DPad L/R)", powerLimit * 100);
        telemetry.addData("Commanded Power", "%.2f  (RT fwd / LT rev)", commandedPower);

        DcMotorEx m = getSelectedMotor();
        if (m != null) {
            telemetry.addLine("--- Motor ---");
            telemetry.addData("Encoder Position", "%d ticks  (Y = reset)", m.getCurrentPosition());
            telemetry.addData("Velocity", "%.0f ticks/sec  (%.0f RPM)", m.getVelocity(), m.getVelocity() / 28.0 * 60.0);
            telemetry.addData("Applied Power", "%.2f", m.getPower());
        } else {
            telemetry.addLine("--- Servo (continuous rotation, no feedback) ---");
        }

        if (selected == LAUNCHER) {
            telemetry.addLine("--- Launcher Options ---");
            telemetry.addData("Mode", "%s  (X = toggle)", launcherVelocityMode ? "VELOCITY" : "POWER");
            if (launcherVelocityMode) {
                telemetry.addData("Target Velocity", "%.0f ticks/sec  (Bumpers -/+ %d)",
                        launcherTargetVelocity, (int) VELOCITY_STEP);
                telemetry.addData("Spin Up", gamepad1.a ? "ACTIVE (A held)" : "Hold A to spin");
            }
        }

        telemetry.addLine("--- Live Snapshot (all motors) ---");
        telemetry.addData("Left Drive", "pos %d | pwr %.2f", leftDrive.getCurrentPosition(), leftDrive.getPower());
        telemetry.addData("Right Drive", "pos %d | pwr %.2f", rightDrive.getCurrentPosition(), rightDrive.getPower());
        telemetry.addData("Intake", "pos %d | pwr %.2f", intake.getCurrentPosition(), intake.getPower());
        telemetry.addData("Launcher", "pos %d | vel %.0f", launcher.getCurrentPosition(), launcher.getVelocity());
        telemetry.addData("Servos L/R/Windmill", "%.2f / %.2f / %.2f",
                leftIntakeServo.getPower(), rightIntakeServo.getPower(), windmillServo.getPower());
        telemetry.update();
    }
}