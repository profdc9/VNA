/* Complex number class */

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

#include "complex.h"

Complex::Complex()
{
}

Complex::Complex(float pReal)
{	
	real = pReal;
	imag = 0.0f;
}

Complex::Complex(float pReal, float pImag)
{
	real = pReal;
	imag = pImag;
}

Complex::Complex(const Complex &cmplx)
{
	real = cmplx.real;
	imag = cmplx.imag;
}

float Complex::re()
{
	return real;
}

float Complex::im()
{
	return imag;
}

float Complex::absq()
{
	return real*real+imag*imag;
}

float Complex::absv()
{
	return sqrtf(real*real+imag*imag);
}

float Complex::arg()
{
	return atan2(imag, real);
}

void Complex::polar(float modulus, float phase)
{
	real = modulus * cosf(phase);
	imag = modulus * sinf(phase);
}

void Complex::polar(float phase)
{
  real = cosf(phase);
  imag = sinf(phase);
}

Complex Complex::recip() const
{
  float mag = real*real+imag*imag;
  return Complex(real/mag,-imag/mag);
}

Complex Complex::parallel(const Complex &cmplx) const
{
   Complex temp = (recip()+cmplx.recip());
   return temp.recip();
}

Complex Complex::operator+(const Complex &val) const
{
	Complex c;
	c.real = real + val.real;
	c.imag = imag + val.imag;
	return c;
}

Complex Complex::operator-(const Complex &val) const
{
	Complex c;
	c.real = real - val.real;
	c.imag = imag - val.imag;
	return c;
}

Complex Complex::operator*(const Complex &val) const
{
	Complex c;
	c.real = real * val.real - imag * val.imag;
	c.imag = real * val.imag + imag * val.real;
	return c;
}

Complex Complex::operator/(const Complex &val) const
{
	Complex c;
	float abssq = 1.0f/(val.real * val.real + val.imag * val.imag);
	c.real = abssq*(real * val.real + imag * val.imag);
	c.imag = abssq*(imag * val.real - real * val.imag);
	return c;
}

Complex Complex::operator+(float val) const
{
	Complex c;
	c.real = real + val;
	c.imag = imag;
	return c;
}

Complex Complex::operator-(float val) const
{
	Complex c;
	c.real = real - val;
	c.imag = imag;
	return c;
}

Complex Complex::operator*(float val) const
{
	Complex c;
	c.real = real * val;
	c.imag = imag * val;
	return c;
}

Complex Complex::operator/(float val) const
{
	Complex c;
	val = 1.0f / val;
	c.real = real * val;
	c.imag = imag * val;
	return c;
}

bool Complex::operator==(const Complex &val)
{
	return ((real == val.real) && (imag == val.imag));
}

bool Complex::operator!=(const Complex &val)
{
	return ((real != val.real) || (imag != val.imag));
}

