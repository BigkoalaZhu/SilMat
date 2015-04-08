#include "MyVector.h"

Sketch2Scene::MyVector::MyVector()
{
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
}

Sketch2Scene::MyVector::MyVector(float x, float y, float z)
{
	data[0] = x;
	data[1] = y;
	data[2] = z;
}

Sketch2Scene::MyVector::MyVector(float* data)
{
	this->data[0] = data[0];
	this->data[1] = data[1];
	this->data[2] = data[2];
}

Sketch2Scene::MyVector::MyVector(const MyVector& o)
{
	data[0] = o[0];
	data[1] = o[1];
	data[2] = o[2];
}

float Sketch2Scene::MyVector::dot(const MyVector o)
{
	return data[0]*o[0]+data[1]*o[1]+data[2]*o[2];
}

Sketch2Scene::MyVector Sketch2Scene::MyVector::cross(const MyVector o)
{
	MyVector result;
	result[0] = data[1] * o[2] - data[2] * o[1];
	result[1] = data[2] * o[0] - data[0] * o[2];
	result[2] = data[0] * o[1] - data[1] * o[0];
	return result;
}

float Sketch2Scene::MyVector::angle(const MyVector o)
{
	MyVector a,b(o);
	a = normalized();
	b.normalize();
	
	return acos(a.dot(b));
}

Sketch2Scene::MyVector Sketch2Scene::MyVector::normalized()
{
	MyVector o(data);
	o.normalize();
	return o;
}

void Sketch2Scene::MyVector::normalize()
{

	float l = length();
	if (abs(l) < 1e-8) {
		if ((data[0] >= data[1]) && (data[0] >= data[2])) {
			data[0] = 1.0f;
			data[1] = data[2] = 0.0f;
		} else
			if (data[1] >= data[2]) {
				data[1] = 1.0f;
				data[0] = data[2] = 0.0f;
			} else {
				data[2] = 1.0f;
				data[0] = data[1] = 0.0f;
			}
	} else {
		float m = 1.0f / l;
		data[0] *= m;
		data[1] *= m;
		data[2] *= m;
	}
}

float Sketch2Scene::MyVector::length()
{
	return sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]);
}

float& Sketch2Scene::MyVector::operator[] (int index)
{
	return data[index];
}

const float& Sketch2Scene::MyVector::operator[] (const int index) const
{
	return data[index];
}

Sketch2Scene::MyVector Sketch2Scene::MyVector::operator+(const MyVector o)
{
	MyVector result;
	result[0] = data[0]+o[0];
	result[1] = data[1]+o[1];
	result[2] = data[2]+o[2];

	return result;
}

Sketch2Scene::MyVector Sketch2Scene::MyVector::operator-(const MyVector o)
{
	MyVector result;
	result[0] = data[0]-o[0];
	result[1] = data[1]-o[1];
	result[2] = data[2]-o[2];

	return result;
}

void Sketch2Scene::MyVector::operator=(const MyVector o)
{
	data[0] = o[0];
	data[1] = o[1];
	data[2] = o[2];
}