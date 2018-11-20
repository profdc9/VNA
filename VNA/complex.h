#ifndef _COMPLEX_H
#define _COMPLEX_H
/*
    Limited complex number class
*/

/*
 * Copyright (c) 2018 Daniel Marks

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */

#include <math.h>

#define DEG2RAD(x) ((x)*(0.0174532925199f))
#define RAD2DEG(x) ((x)*(57.2957795131f))

class Complex
{
	public:
	
	float real, imag;
	
	Complex();
	Complex(float pReal);
	Complex(float pReal, float pImag);
	Complex(const Complex &cmplx);
	float re();
	float im();
	float absq();
	void polar(float modulus, float phase);
  void polar(float phase);
	float arg();
	float absv();
  Complex recip() const;
  Complex parallel(const Complex &cmplx) const;
	
	Complex operator+(const Complex &val) const;
	Complex operator-(const Complex &val) const;
	Complex operator*(const Complex &val) const;
	Complex operator/(const Complex &val) const;
	Complex operator+(float val) const;
	Complex operator-(float val) const;
	Complex operator*(float val) const;
	Complex operator/(float val) const;
	bool operator==(const Complex &val);
	bool operator!=(const Complex &val);
};

#endif  /* _COMPLEX_H */
