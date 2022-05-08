package com.ThermalEquilibrium.homeostasis.Filters.Estimators;


import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;


/**
 * When you have good sensors.
 */
public class RawValue extends Estimator {
	/**
	 * Set up Double Supplier for recurring measurement obtainment.
	 *
	 * @param measurement measurement we want to obtain.
	 */
	public RawValue(DoubleSupplier measurement) {
		super(measurement);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public double update() {
		return measurement.getAsDouble();
	}
}
