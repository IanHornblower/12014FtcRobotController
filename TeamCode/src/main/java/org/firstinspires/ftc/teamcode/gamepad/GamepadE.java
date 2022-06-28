package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Timer;

public class GamepadE {

    public Gamepad gamepad;

    Timer toggleTimer = new Timer();

    private double toggleDelay = 0.2;

    public boolean leftBumper, rightBumper, dpadLeft, dpadUp, dpadRight, dpadDown, square, triangle, circle, cross, leftJoystickButton, rightJoystickButton;
    public double leftTrigger, rightTrigger, leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;

    public boolean TleftBumper = false, TrightBumper = false, TdpadLeft = false, TdpadUp = false, TdpadRight = false, TdpadDown = false, Tsquare = false, Ttriangle = false, Tcircle = false, Tcross = false, TleftJoystickButton = false, TrightJoystickButton = false;

    public boolean[] buttons = {leftBumper, rightBumper, dpadLeft, dpadUp, dpadRight, dpadDown, square, triangle, circle, cross, leftJoystickButton, rightJoystickButton};
    public double[] analogs = {leftTrigger, rightTrigger, leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY};

    public enum Buttons {
        leftBumper,
        rightBumper,
        dpadLeft,
        dpadRight,
        dpadDown,
        dpadUp,
        square,
        triangle,
        circle,
        cross,
        x,
        y,
        b,
        a,
        leftJoystickButton,
        rightJoystickButton
    }

    enum Analogs {
        leftTrigger,
        rightTrigger,
        leftJoystickX,
        leftJoystickY,
        rightJoystickX,
        rightJoystickY
    }

    public static enum DPAD {
        left,
        right,
        up,
        down,
        unpressed
    }

    public static enum BUMPER {
        left,
        right,
        both,
        unpressed
    }

    public GamepadE(Gamepad gamepad) {
        this.gamepad = gamepad;

        for(boolean button : buttons) {
            button = false;
        }
        for(double analog : analogs) {
            analog = 0.0;
        }
    }

    public Gamepad gamepad() {
        return gamepad;
    }

    public void setButtonToggle(Buttons buttons) {
        switch (buttons) {
            case leftBumper:
                TleftBumper = true;
                break;

            case rightBumper:
                TrightBumper = true;
                break;

            case dpadLeft:
                TdpadLeft = true;
                break;

            case dpadRight:
                TdpadRight = true;
                break;

            case dpadDown:
                TdpadDown = true;
                break;

            case dpadUp:
                TdpadUp = true;
                break;

            case x:
            case square:
                Tsquare = true;
                break;

            case y:
            case triangle:
                Ttriangle = true;
                break;

            case b:
            case circle:
                Tcircle = true;
                break;

            case a:
            case cross:
                Tcross = true;
                break;

            case leftJoystickButton:
                TleftJoystickButton = true;
                break;

            case rightJoystickButton:
                TrightJoystickButton = true;
                break;
        }
    }

    public GamepadEx.DPAD dpad() {
        if(dpadUp) return GamepadEx.DPAD.up;
        else if(dpadDown) return GamepadEx.DPAD.down;
        else if(dpadLeft) return GamepadEx.DPAD.left;
        else if(dpadRight) return GamepadEx.DPAD.right;
        else return GamepadEx.DPAD.unpressed;
    }

    public GamepadEx.BUMPER bumpers() {
        if(leftBumper) return GamepadEx.BUMPER.left;
        else if(rightBumper) return GamepadEx.BUMPER.right;
        else if(rightBumper && leftBumper) return GamepadEx.BUMPER.both;
        else return GamepadEx.BUMPER.unpressed;
    }

    public void rumble(int duration) {
        gamepad.rumble(duration);
    }

    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    public void init() {
        toggleTimer.start();
    }

    public void update() {
        if(TleftBumper) {
            if(gamepad.left_bumper && toggleTimer.currentSeconds() >= toggleDelay) {
                leftBumper = !leftBumper;
                toggleTimer.reset();
            }
        }
        else leftBumper = gamepad.left_bumper;

        if(TrightBumper) {
            if(gamepad.right_bumper && toggleTimer.currentSeconds() >= toggleDelay) {
                rightBumper = !rightBumper;
                toggleTimer.reset();
            }
        }
        else rightBumper = gamepad.right_bumper;

        if(TdpadLeft) {
            if(gamepad.dpad_left && toggleTimer.currentSeconds() >= toggleDelay) {
                dpadLeft = !dpadLeft;
                toggleTimer.reset();
            }
        }
        else dpadLeft = gamepad.dpad_left;

        if(TdpadUp) {
            if(gamepad.dpad_up && toggleTimer.currentSeconds() >= toggleDelay) {
                dpadUp = !dpadUp;
                toggleTimer.reset();
            }
        }
        else dpadUp = gamepad.dpad_up;

        if(TdpadDown) {
            if(gamepad.dpad_down && toggleTimer.currentSeconds() >= toggleDelay) {
                dpadDown = !dpadDown;
                toggleTimer.reset();
            }
        }
        else dpadDown = gamepad.dpad_down;

        if(TdpadRight) {
            if(gamepad.dpad_right && toggleTimer.currentSeconds() >= toggleDelay) {
                dpadRight = !dpadRight;
                toggleTimer.reset();
            }

        }
        else dpadRight = gamepad.dpad_right;

        if(TdpadLeft) {
            if(gamepad.dpad_left && toggleTimer.currentSeconds() >= toggleDelay) {
                dpadLeft = !dpadLeft;
                toggleTimer.reset();
            }
        }
        else dpadLeft = gamepad.dpad_left;

        if(Tsquare) {
            if(gamepad.square && toggleTimer.currentSeconds() >= toggleDelay) {
                square = !square;
                toggleTimer.reset();
            }

        }
        else square = gamepad.square;

        if(Ttriangle) {
            if(gamepad.triangle && toggleTimer.currentSeconds() >= toggleDelay) {
                triangle = !triangle;
                toggleTimer.reset();
            }

        }
        else triangle = gamepad.triangle;

        if(Tcircle) {
            if(gamepad.circle && toggleTimer.currentSeconds() >= toggleDelay) {
                circle = !circle;
                toggleTimer.reset();
            }
        }
        else circle = gamepad.circle;

        if(Tcross) {
            if (gamepad.cross && toggleTimer.currentSeconds() >= toggleDelay) {
                cross = !cross;
                toggleTimer.reset();
            }
        }
        else cross = gamepad.cross;

        if(TleftJoystickButton) {
            if(gamepad.left_stick_button && toggleTimer.currentSeconds() >= toggleDelay) {
                leftJoystickButton = !leftJoystickButton;
                toggleTimer.reset();
            }
        }
        else leftJoystickButton = gamepad.left_stick_button;

        if(TrightJoystickButton) {
            if(gamepad.right_stick_button && toggleTimer.currentSeconds() >= toggleDelay) {
                rightJoystickButton = !rightJoystickButton;
                toggleTimer.reset();
            }
        }
        else rightJoystickButton = gamepad.right_stick_button;

        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;
        leftJoystickY = gamepad.left_stick_y;
        leftJoystickX = gamepad.left_stick_x;
        rightJoystickY = gamepad.right_stick_y;
        rightJoystickX = gamepad.right_stick_x;
    }
}
