#ifndef MYVECTOR_H
#define MYVECTOR_H

#include<cmath>

using namespace std;

namespace Sketch2Scene
{
	class MyVector
	{
	public:
		MyVector();
		MyVector(float x, float y, float z);
		MyVector(float* data);
		MyVector(const MyVector& o);

		float dot(const MyVector o);
		MyVector cross(const MyVector o);
		float angle(const MyVector o);
		MyVector normalized();
		void normalize();
		float length();

		float& operator[] (const int index);
		const float& operator[] (const int index) const;
		void operator=(const MyVector o);
		MyVector operator+(const MyVector o);
		MyVector operator-(const MyVector o);

	private:
		float data[3];
	};
}

#endif