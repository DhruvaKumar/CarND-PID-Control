#include "PID.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}
PID::PID(std::string name): name(name) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	// twiddle
	autotune = false;
  	N_total_steps = 600; // total steps to execute for a twiddle run
  	threshold_step_to_record = 100; // steps after which twiddle should start recording error
  	reset_sim = false; // flag to notify main to reset sim after a twiddle run
  	twiddle_error = 0.0;
  	best_twiddle_error = 100000.0;
	dp = {0.5*Kp, 0.5*Kd, 0.5*Ki};
	index = 0; // 0:Kp, 1:Kd, 2:Ki
	twiddle_step = 0;
	tried_increasing = false;
	tried_decreasing = false;
	tolerance = 0.01;
	i_error = 0.0;
	step = 0;
	
}

void PID::UpdateError(double cte) 
{
	// prev cte for inital step
	if (step == 0)
		p_error = cte;

	d_error = cte - p_error;

	p_error = cte;

	i_error += cte;
	// reset integral error if cte is close to 0
	if (fabs(cte) < 0.0001)
		i_error = 0.0;

	// prevent integral windup
	if (i_error > 100.0)
		i_error = 100.0;


	// twiddle
	if (autotune)
	{
		// accumulate error for this twiddle run
		if (step > threshold_step_to_record && step <= N_total_steps)
		{
			twiddle_error += cte * cte;
		}

		if (step >= N_total_steps)
		{
			auto avg_twiddle_error = twiddle_error / (N_total_steps - threshold_step_to_record);
			// auto avg_twiddle_error = twiddle_error;

			twiddlef.open("twiddle.txt", std::ios_base::app);
			twiddlef << name << " twiddle step=" << twiddle_step << "\n";
			twiddlef << "avg_twiddle_error=" << avg_twiddle_error << "\n";
			twiddlef << "best_twiddle_error=" << best_twiddle_error << "\n";
  			twiddlef << "Kp=" << Kp << " Kd=" << Kd << " Ki=" << Ki << "\n";
  			twiddlef << "dp0=" << dp[0] << " dp1=" << dp[1] << " dp2=" << dp[2] << "\n";
			twiddlef << "tried_increasing=" << tried_increasing << "\n";
			twiddlef << "tried_decreasing=" << tried_decreasing << "\n";
			twiddlef << "--------------\n";
			twiddlef.close();

			// set best error for initial twiddle run
			if (twiddle_step == 0)
				best_twiddle_error = avg_twiddle_error;
			else
			{
				// record best error, increase dp and move on to next index 
				if (avg_twiddle_error < best_twiddle_error)
				{
					best_twiddle_error = avg_twiddle_error;
					dp[index] *= 1.1;
					index = (index+1) % 3;
					tried_increasing = false;
					tried_decreasing = false;
				}
			}

			if (!tried_increasing && !tried_decreasing)
			{
				IncrementK(index, dp[index]);
				tried_increasing = true;
			}
			else if (tried_increasing)
			{
				tried_increasing = false;
				IncrementK(index, -2*dp[index]);
				tried_decreasing = true;
			}
			else if (tried_decreasing)
			{
				tried_decreasing = false;
				IncrementK(index, dp[index]);
				dp[index] *= 0.9;
				index = (index+1) % 3;

				IncrementK(index, dp[index]);
				tried_increasing = true;
			}

			// reset sim
			step = 0;
			twiddle_error = 0.0;
			twiddle_step++;
			reset_sim = true;

			// stop tuning once sum of dp < tol
			if (std::accumulate(dp.begin(), dp.end(), 0) < tolerance)
			{
				autotune = false;
			}
			return;
		}
	}

	step++;
}

double PID::TotalError() 
{
	return (Kp*p_error + Ki*i_error + Kd*d_error);
}

void PID::IncrementK(int index, double dp)
{
	if (index == 0)
	{
		Kp += dp;
	}
	else if (index == 1)
	{
		Kd += dp;
	}
	else if (index == 2)
	{
		Ki += dp;
	}
}

