#ifndef QUARTIC_H
#define QUARTIC_H

#include "FPUtil.h"
#include <vector>
#include <assert.h>
#include <math.h>


inline bool isBiquadratic(std::vector<double> const& v) {
  return fequal(v[1],0.0) && fequal(v[3],0.0);
}

void solveQuadratic(double a, double b, double c, std::vector<double>& out) {
	if(a==0){
		out.reserve(1);
		if(b==0){
			out.push_back(c);
			return;
		}
		out.push_back(-c/b);
		return;
	}
	double d(b*b-4*a*c);
	if(d<0) return; // No real roots
	out.reserve(2);
	out.push_back((-b-sqrt(d))/(2*a));
	out.push_back((-b+sqrt(d))/(2*a));
}

void solveCubic(double a, double b, double c, double d, std::vector<double>& out){
	if(a == 0){
		solveQuadratic(b,c,d,out);
		return;
	}
	if(d == 0){
		solveQuadratic(a,b,c,out);
		out.push_back(0);
		return;
	}
	b /= a;
	c /= a;
	d /= a;
	double bb(b*b);
	double q((3.0*c - (bb))/9.0);
	double r((-(27.0*d) + b*(9.0*c - 2.0*(bb)))/54.0);
	double disc(q*q*q + r*r);
	double term1(b/3.0);

	// one root real, two are complex
	if(disc > 0) {
		out.reserve(1);
		double s = r + sqrt(disc);
		s = ((s < 0) ? -pow(-s, (1.0/3.0)) : pow(s, (1.0/3.0)));
		double t = r - sqrt(disc);
		t = ((t < 0) ? -pow(-t, (1.0/3.0)) : pow(t, (1.0/3.0)));
		out.push_back(-term1 + s + t);
		// Ignore complex roots
		return;
	} 

	// The remaining options are all real
	if(disc == 0){ // All roots real, two are equal.
		out.reserve(2);
		double r13 = ((r < 0) ? -pow(-r,(1.0/3.0)) : pow(r,(1.0/3.0)));
		out.push_back(-term1 + 2.0*r13);
		out.push_back(-(r13 + term1)); // this is a double root (just report it once)
		return;
	}

	// Only option left is that all roots are real and unequal (to get here, q < 0)
	q = -q;
	double dum1 = q*q*q;
	dum1 = acos(r/sqrt(dum1));
	double r13 = 2.0*sqrt(q);
	out.reserve(3);
	out.push_back(-term1 + r13*cos(dum1/3.0));
	out.push_back(-term1 + r13*cos((dum1 + 2.0*M_PI)/3.0));
	out.push_back(-term1 + r13*cos((dum1 + 4.0*M_PI)/3.0));
	return;
}

double findFerrariY(std::vector<double> const& coeffs) {
	double const& c(coeffs[2]);
	double const& d(coeffs[3]);
	double const& e(coeffs[4]);
	double cc(c*c);
	double a3 = 1.0;
	double a2 = 5.0 / 2.0 * c;
	double a1 = 2.0 * cc - e;
	double a0 = cc*c / 2.0 - c * e / 2.0 - d*d / 8.0;

	std::vector<double> roots;
	solveCubic(a3,a2,a1,a0,roots);

	for(double y : roots) {
		if(!fequal(c + 2.0 * y,0.0)){
			return y;
		}
	}
	assert(!"Ferrari method should have at least one y");
}

void solveBiquadratic(std::vector<double> const& in, std::vector<double>& out) {
	double const& a=in[0];
	double const& b=in[2];
	double const& c=in[4];

	std::vector<double> quadRoots;
	solveQuadratic(a,b,c,quadRoots);

	if(quadRoots.size()==2&&quadRoots.front()==quadRoots.back()){
		quadRoots.erase(quadRoots.begin()+1);
	}
	for (double quadRoot : quadRoots) {
		if (quadRoot > 0) {
			double tmp(sqrt(quadRoot));
			out.push_back(tmp);
			out.push_back(-tmp);
		} else if (quadRoot == 0) {
			out.push_back(0);
		}
	}
}

void toDepressed(std::vector<double> const& in, std::vector<double>& coeffs) {
	// http://en.wikipedia.org/wiki/Quartic_function#Converting_to_a_depressed_quartic
	double const& a(in[0]);
	double const& b(in[1]);
	double const& c(in[2]);
	double const& d(in[3]);
	double const& e(in[4]);
	double aa(a*a);
	double bb(b*b);
	double ac(a*c);
	double p = (8.0 * ac - 3.0 * bb) / (8.0 * aa);
	double q = (bb*b - 4.0 * ac * b + 8.0 * d * aa) / (8.0 * aa*a);
	double r = (-3.0 * bb*bb + 256.0 * e * aa*a - 64.0 * d * b * aa + 16.0 * ac * bb)/(256.0 * aa*aa);
	//double p = (8.0 * a * c - 3.0 * pow(b, 2.0)) / (8.0 * pow(a, 2.0));
	//double q = (pow(b, 3.0) - 4.0 * a * b * c + 8.0 * d * pow(a, 2.0)) / (8.0 * pow(a, 3.0));
	//double r = (-3.0 * pow(b, 4.0) + 256.0 * e * pow(a, 3.0) - 64.0 * d * b * pow(a, 2.0) + 16.0 * c
	//* a * pow(b, 2.0)) / (256.0 * pow(a, 4.0));
	coeffs.reserve(5);
	coeffs.push_back(1);
	coeffs.push_back(0);
	coeffs.push_back(p);
	coeffs.push_back(q);
	coeffs.push_back(r);
	return;
}

void solveUsingFerrariMethodWikipedia(std::vector<double> const& in, std::vector<double>& out) {
	// http://en.wikipedia.org/wiki/Quartic_function#Ferrari.27s_solution
	double const& a(in[0]);
	double const& b(in[1]);
	std::vector<double> coeffs;
	toDepressed(in,coeffs);
	if(isBiquadratic(coeffs)) {
		std::vector<double> biquad;
		solveBiquadratic(coeffs,biquad);
		// Convert back to original roots
		out.reserve(biquad.size());
		for (auto const& root:biquad){
			out.push_back(root-b/(4.0 * a));
		}
		return;
	}

	double y = findFerrariY(coeffs);
	double originalRootConversionPart = -b / (4.0 * a);
	double const& dc(coeffs[2]);
	double const& dd(coeffs[3]);
	double firstPart = sqrt(dc + 2.0 * y);
	double f(3.0*dc+2.0*y);
	double s(2.0*dd/sqrt(dc + 2.0 * y));

	double f1(-(f+s));
	double f2(-(f-s));
	if(f1>=0){
		double secondPart(sqrt(f1));
		out.push_back(originalRootConversionPart + (firstPart + secondPart) / 2.0);
		out.push_back(originalRootConversionPart + (firstPart - secondPart) / 2.0);
	}
	if(f2>=0){
		double secondPart(sqrt(f2));
		out.push_back(originalRootConversionPart + (-firstPart + secondPart) / 2.0);
		out.push_back(originalRootConversionPart + (-firstPart - secondPart) / 2.0);
	}
}

void findRealRoots(std::vector<double> const& in, std::vector<double>& out) {
	if(in.size()==5){
		if (fequal(in[0],0)) {
			if (fequal(in[1],0)) {
				solveQuadratic(in[2],in[3],in[4],out);
				return;
			}
			solveCubic(in[1],in[2],in[3],in[4],out);
			return;
		}else if(fequal(in[4],0)){
			solveCubic(in[0],in[1],in[2],in[3],out);
			out.push_back(0);
			return;
		}

		if (isBiquadratic(in)) {
			solveBiquadratic(in,out);
			return;
		}
		solveUsingFerrariMethodWikipedia(in,out);
		return;
	}else if(in.size()==4){
		solveCubic(in[0],in[1],in[2],in[3],out);
		return;
	}else if(in.size()==3){
		solveQuadratic(in[0],in[1],in[2],out);
		return;
	}else if(in.size()==2){
		solveQuadratic(0,in[0],in[1],out);
		return;
	}else if(in.size()==1){
		solveQuadratic(0,0,in[0],out);
		return;
	}
	assert(!"Polynomial requested has too many (or zero) coefficients. [5 max]");
}

#endif
