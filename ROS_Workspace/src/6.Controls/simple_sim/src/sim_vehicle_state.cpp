#include "sim_vehicle_state.hpp"

namespace sim
{
/* STATE */
double State::calc_sa_r(double v_y, double r, double v_x) const
{
	if (v_x == 0)
		return 0;
	double lr = calc_lr(simplified, this->a_x);

	return std::atan((v_y - lr * r) / v_x);
}
double State::calc_sa_f(double v_y, double r, double v_x, double d) const
{
	if (v_x == 0)
		return 0;
	double lf = calc_lf(calc_lr(simplified, this->a_x));

	return std::atan((v_y + lf * r) / v_x) - d;
}
double sign(double x)
{
	return x == 0 ? 0 : x / std::abs(x);
}
double State::calc_fry(double sa_r, double frz) const
{
	const double P[] = {-1.9391836, 2.46810824, -0.21654031,
						5.53792739, 0.29777483, -0.87901614,
						-0.0990718, 7.02929697, -29.59686021,
						0.97520799, -8.20731378, 0.5275139,
						-1.0086604, -0.61898062, -55.82849645,
						70.38725604, 4.67966457, 0.9878318};
	const double ex = 0.001;
	const double Fz0 = 1112.0554070627252;
	double dfz = (frz - Fz0) / Fz0;
	const double IA = 0;
	double D = (abs(P[1]) + P[2] * dfz) * frz / (1 + P[3] * IA * IA);
	double C = abs(P[0]);
	double SVg = frz * (P[12] + P[13] * dfz) * IA;
	double Kg = frz * (P[16] + P[17] * dfz);
	double BCD = P[8] * Fz0 * std::sin(2.0 * std::atan(frz / ((P[9] + P[15] * IA * IA) * Fz0))) / (1.0 + P[10] * IA * IA);
	double Sh = (Kg * IA - SVg) / (BCD + ex);
	double ay = sa_r + Sh;
	double E = (P[4] + P[5] * dfz) * (1.0 + P[14] * IA * IA - (P[6] + P[7] * IA) * sign(ay));
	double B = BCD / (C * D + ex);
	double Sv = frz * (0 + 0 * dfz) + SVg;
	double psi = D * std::sin(C * std::atan(B * ay - E * (B * ay - std::atan(B * ay))));

	return psi + Sv;
}
double State::calc_ffy(double sa_r, double ffz) const
{
	const double P[] = {-1.9391836, 2.46810824, -0.21654031,
						5.53792739, 0.29777483, -0.87901614,
						-0.0990718, 7.02929697, -29.59686021,
						0.97520799, -8.20731378, 0.5275139,
						-1.0086604, -0.61898062, -55.82849645,
						70.38725604, 4.67966457, 0.9878318};
	const double ex = 0.001;
	const double Fz0 = 1112.0554070627252;
	double dfz = (ffz - Fz0) / Fz0;
	const double IA = 0;
	double D = (abs(P[1]) + P[2] * dfz) * ffz / (1 + P[3] * IA * IA);
	double C = abs(P[0]);
	double SVg = ffz * (P[12] + P[13] * dfz) * IA;
	double Kg = ffz * (P[16] + P[17] * dfz);
	double BCD = P[8] * Fz0 * std::sin(2.0 * std::atan(ffz / ((P[9] + P[15] * IA * IA) * Fz0))) / (1.0 + P[10] * IA * IA);
	double Sh = (Kg * IA - SVg) / (BCD + ex);
	double ay = sa_r + Sh;
	double E = (P[4] + P[5] * dfz) * (1.0 + P[14] * IA * IA - (P[6] + P[7] * IA) * sign(ay));
	double B = BCD / (C * D + ex);
	double Sv = ffz * (0 + 0 * dfz) + SVg;
	double psi = D * std::sin(C * std::atan(B * ay - E * (B * ay - std::atan(B * ay))));

	return psi + Sv;
}
double State::calc_ffz(double v_x) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);

	return constants.m * constants.g * lf / (lf + lr) + 0.25 * constants.P_air * constants.ClA * v_x * v_x;
}
double State::calc_frz(double v_x) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);

	return constants.m * constants.g * lr / (lf + lr) + 0.25 * constants.P_air * constants.ClA * v_x * v_x;
}
double State::calc_lr(bool simplified, double a_x) const
{
	a_x = simplified ? 0 : a_x / 9.81;
	double w = constants.m;
	double dwx = constants.h_cog * w * a_x / constants.wheelbase;
	double wf = w * constants.wd - dwx;

	return constants.wheelbase * (wf / w); 
}
double State::calc_lf(double l_r) const
{
	return constants.wheelbase - l_r;
}
double State::calc_drag(double v_x) const
{
	return 0.5 * constants.P_air * constants.CdA * v_x * v_x;
}
double State::calc_roll(double fz) const
{
	double coeff = 0.03;

	return coeff * fz;
}
double State::v_x_next(double v_x, double a_x, double dt) const
{
	return std::max(0.0, v_x + a_x * dt);
}
double State::v_y_next(double v_y, double a_y, double dt) const
{
	return v_y + a_y * dt;
}
double State::calc_a_x(double frx, double ffx, double f_drag, double f_roll, double ffy, double d, double v_y, double r) const
{
	double ax_print = (frx + ffx - f_drag - f_roll - ffy * std::sin(d)) / constants.m + v_y * r;

	return (frx + ffx - f_drag - f_roll - ffy * std::sin(d)) / constants.m + v_y * r;	
}
double State::calc_a_y(double fry, double ffx, double ffy, double d, double v_x, double r) const
{
	return (fry + ffy * std::cos(d)) / constants.m - v_x * r; //- ffx*std::sin(d)
}
double State::r_next(double ffy, double ffx, double d, double fry, double r, double dt) const
{
	double lr = calc_lr(simplified, a_x);
	double lf = calc_lf(lr);
	double r_dot = (ffy * lf * std::cos(d) - fry * lr) / constants.Iz; //+ffx*lf*std::sin(d)

	return r + r_dot * dt;
}
double State::theta_next(double r, double theta, double dt) const
{
	return theta + r * dt;
}
double State::x_next(double v_x, double theta, double v_y, double x, double dt) const
{
	double x_dot = v_x * std::cos(theta) - v_y * std::sin(theta);

	return x + x_dot * dt;
}
double State::y_next(double v_x, double theta, double v_y, double y, double dt) const
{
	double y_dot = v_x * std::sin(theta) + v_y * std::cos(theta);

	return y + y_dot * dt;
}
double State::s_next(double s, double v_x, double v_y, double a_x, double a_y, double dt) const
{
	double v = std::sqrt(v_x * v_x + v_y * v_y);
	double a = std::sqrt(a_x * a_x + a_y * a_y);

	return s + v * dt + 0.5 * a * dt * dt;
}

void State::next(double dt, double frx, double ffx, double delta)
{
	this->frx = frx;
	this->ffx = ffx;

	d = delta;

	// Calculate slip angles
	sa_r = calc_sa_r(v_y, r, v_x);
	sa_f = calc_sa_f(v_y, r, v_x, delta);

	// Calculate forces on z-axis
	ffz = calc_ffz(v_x);
	frz = calc_frz(v_x);

	// Calculate forces on y-axis
	fry = calc_fry(sa_r, frz);
	ffy = calc_ffy(sa_f, ffz);

	// Calculate components of x-axis forces
	double f_drag = calc_drag(v_x);
	double f_roll = calc_roll(frz + ffz);

	// Calculate second derivatives
	a_x = calc_a_x(frx, ffx, f_drag, f_roll, ffy, delta, v_y, r);
	a_y = calc_a_y(fry, ffx, ffy, delta, v_x, r);

	// Calculate first derivatives
	v_x = v_x_next(v_x, a_x, dt);
	v_y = v_y_next(v_y, a_y, dt);
	r = r_next(ffy, ffx, delta, fry, r, dt);

	// Calculate base variables
	x = x_next(v_x, theta, v_y, x, dt);
	y = y_next(v_x, theta, v_y, y, dt);
	theta = theta_next(r, theta, dt);
	s = s_next(s, v_x, v_y, a_x, a_y, dt);
	
	// Update time
	t += dt;
}

std::pair<double, double> State::get_mx_my(double vx, double ax)
{
	// Full vehicle
	const double Fz = 193.5 * 9.81 * 0.467 - 0.28 * 193.5 * ax / 1.59  + 0.5 * 1.225 * 2 * vx * vx;
	const double Fz0 = 1112.0554070627252;
	const double dFz = (Fz - Fz0) / Fz0;
	const double mx_max = 0.66 * (2.21891927 - 1.36151651e-07 * dFz);
	const double my_max = 0.66 * (2.46810824 - 0.21654031 * dFz);
	return std::make_pair(mx_max, my_max);
}

std::ostream &operator<<(std::ostream &out, const State &a)
{
	out << std::setw(7) << "t"
		<< /*"\t"*/ " " << std::setw(7) << "v_x"
		<< /*"\t"*/ " " << std::setw(7) << "v_y"
		<< /*"\t"*/ " " << std::setw(7) << "a_x"
		<< /*"\t"*/ " " << std::setw(7) << "a_y"
		<< /*"\t"*/ " " << std::setw(7) << "r"
		<< /*"\t"*/ " " << std::setw(7) << "sa_f"
		<< /*"\t"*/ " " << std::setw(7) << "sa_r"
		<< /*"\t"*/ " " << std::setw(7) << "ffy"
		<< /*"\t"*/ " " << std::setw(7) << "fry"
		<< /*"\t"*/ " " << std::setw(7) << "x"
		<< /*"\t"*/ " " << std::setw(7) << "y"
		<< /*"\t"*/ " " << std::setw(7) << "theta"
		<< /*"\t"*/ " " << std::setw(7) << "s"
		<< std::endl;
	out << std::setw(7) << a.t
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.v_x
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.v_y
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.a_x
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.a_y
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.r
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.sa_f
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.sa_r
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.ffy
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.fry
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.x
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.y
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.theta
		<< /*"\t"*/ " " << std::fixed << std::setprecision(3) << std::setw(7) << a.s
		<< std::endl;
	return out;																																				
}


}