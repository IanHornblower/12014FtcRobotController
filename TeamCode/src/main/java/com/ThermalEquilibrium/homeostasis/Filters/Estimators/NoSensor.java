package com.ThermalEquilibrium.homeostasis.Filters.Estimators;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;

public class NoSensor extends Estimator{


	@RequiresApi(api = Build.VERSION_CODES.N)
	public NoSensor() {

		super(new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return 0;
			}
		});
	}

	@Override
	public double update() {
		return 0;
	}
}
