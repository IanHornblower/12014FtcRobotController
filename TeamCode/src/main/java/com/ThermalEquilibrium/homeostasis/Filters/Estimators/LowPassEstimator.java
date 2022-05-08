package com.ThermalEquilibrium.homeostasis.Filters.Estimators;


import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;

import java.util.function.DoubleSupplier;


public class LowPassEstimator extends Estimator{

	protected LowPassFilter filter;

	/**
	 * Set up Double Supplier for recurring measurement obtainment.
	 *
	 * Uses a low pass filter to estimate the systems state.
	 *
	 * @param measurement measurement we want to obtain.
	 */
	public LowPassEstimator(DoubleSupplier measurement, double LowPassGain) {
		super(measurement);
		filter = new LowPassFilter(LowPassGain);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public double update() {
		return filter.estimate(measurement.getAsDouble());
	}
}
