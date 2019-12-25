#pragma once

#include <cmath>
#include <iostream>

#define PI 3.141592653589793
#define kEpsilon .000000001

using namespace std;

    inline double toRadians(double degrees) {
        return degrees * (PI / 180.0);
    }

    inline double toDegrees(double degrees) {
        return degrees * (180.0 / PI);
    }

    inline double arctan(double x, double y) {
        double result = toDegrees(atan2(y, x));
        if(x >= 0 && y >= 0) {
            return 90 - result;
        } else if(x >= 0 && y < 0) {
            return 90 + -1 * result;
        } else if(x <= 0 && y < 0) {
            return 90 + -1 * result;
        } else if(x < 0 && y >= 0) {
            return 450 - result;
        }
        return -1;
    }

	inline double pathogram(double x, double y){
		return sqrt(x*x + y*y);
	}

