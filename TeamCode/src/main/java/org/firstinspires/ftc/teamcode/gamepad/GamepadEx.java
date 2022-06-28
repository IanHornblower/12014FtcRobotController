package org.firstinspires.ftc.teamcode.gamepad;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

public class GamepadEx {

    private final Gamepad gamepad;
    public Joystick leftJoystick;
    public Joystick rightJoystick;
    public Trigger leftTrigger;
    public Trigger rightTrigger;

    public Button leftBumper, rightBumper, dpadLeft, dpadUp, dpadRight, dpadDown, square, triangle, circle, cross;
    public Button[] buttons;

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

    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.leftJoystick = new Joystick(this, Joystick.SIDE.left);
        this.rightJoystick = new Joystick(this, Joystick.SIDE.right);
        this.leftTrigger = new Trigger(this, Trigger.SIDE.left);
        this.rightTrigger = new Trigger(this, Trigger.SIDE.right);

        this.leftBumper = new Button(()-> gamepad.left_bumper);
        this.rightBumper = new Button(()-> gamepad.right_bumper);
        this.dpadLeft = new Button(()-> gamepad.dpad_left);
        this.dpadRight = new Button(()-> gamepad.dpad_right);
        this.dpadUp = new Button(()-> gamepad.dpad_up);
        this.dpadDown = new Button(()-> gamepad.dpad_down);
        this.square = new Button(()-> gamepad.square);
        this.triangle = new Button(()-> gamepad.triangle);
        this.circle = new Button(()-> gamepad.circle);
        this.cross = new Button(()-> gamepad.cross);

        buttons = new Button[]{leftBumper, rightBumper, dpadLeft, dpadUp, dpadRight, dpadDown, square, triangle, circle, cross};
    }

    public Gamepad gamepad() {
        return gamepad;
    }

    public void init() {

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void update() {
        for(Button button : buttons) {
            button.update();
        }
    }

    /*
          // Buttons //
     */

    public boolean cross() {
        return cross.isPressed();
    }

    public boolean circle() {
        return circle.isPressed();
    }

    public boolean square() {
        return square.isPressed();
    }

    public boolean triangle() {
        return triangle.isPressed();
    }

    /*
            // Dpad //
     */

    public boolean dpad_left() {
        return dpadLeft.isPressed();
    }

    public boolean dpad_right() {
        return dpadRight.isPressed;
    }

    public boolean dpad_up() {
        return dpadUp.isPressed();
    }

    public boolean dpad_down() {
        return dpadDown.isPressed();
    }

    public DPAD dpad() {
        if(dpad_up()) return DPAD.up;
        else if(dpad_down()) return DPAD.down;
        else if(dpad_left()) return DPAD.left;
        else if(dpad_right()) return DPAD.right;
        else return DPAD.unpressed;
    }

    /*
            // Bumpers //
     */

    public boolean left_bumper() {
        return leftBumper.isPressed();
    }

    public boolean right_bumper() {
        return rightBumper.isPressed();
    }

    public BUMPER bumpers() {
        if(left_bumper()) return BUMPER.left;
        else if(right_bumper()) return BUMPER.right;
        else if(right_bumper() && left_bumper()) return BUMPER.both;
        else return BUMPER.unpressed;
    }

    /*
            // Rumble //
     */

    public void rumble(int duration) {
        gamepad.rumble(duration);
    }

    public void rumbleBlips(int count) {
        gamepad.rumbleBlips(count);
    }

    public String toString(String gamepad) {
        String title = gamepad + ": \n";
        String leftJoysticks = "Left Joystick: x: " +   roundPlaces(leftJoystick.x(), 2) +  "  y: " + roundPlaces(leftJoystick.y(), 2);
        String rightJoysticks = "Right Joystick: x: " + roundPlaces(rightJoystick.x(), 2) + "  y: " + roundPlaces(rightJoystick.y(), 2);
        String triggers = "Left Trigger: " + leftTrigger.howPressed().toString() + " value: " + roundPlaces(leftTrigger.value(), 2) + "  Right Trigger: " + rightTrigger.howPressed().toString() + " value: " + roundPlaces(rightTrigger.value(), 2);

        return title + leftJoysticks + "\n" + rightJoysticks + "\n" + triggers;
    }

}
