package org.firstinspires.ftc.teamcode.gamepad;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.BooleanSupplier;

public class Button extends Part {

    boolean isPressed = false;
    boolean isToggle = false;

    BooleanSupplier button;

    public Button(BooleanSupplier button) {
        this.button = button;
    }

    public boolean isPressed() {
        return isPressed;
    }

    public void setToggle() {
        isToggle = true;
    }

    public void removeToggle() {
        isToggle = false;
    }

    public boolean isToggle() {
        return isToggle;
    }

    @Override
    public void init() {

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void update() {
        if(isToggle) {
            if(button.getAsBoolean() && toggleTimer.currentSeconds() >= toggleDelay) {
                isPressed = !isPressed;
                toggleTimer.reset();
            }
        }
        else isPressed = button.getAsBoolean();
    }
}
