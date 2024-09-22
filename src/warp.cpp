/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tent(float u) { return (u < 0.5f) ? sqrt(2.0f * u) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * u);}
float tentPdf(float u) { return (u >= -1 && u <= 1) ? 1 - abs(u) : 0; }

Point2f Warp::squareToTent(const Point2f &sample) {
    return Point2f(tent(sample.x()), tent(sample.y()));
}

float Warp::squareToTentPdf(const Point2f &p) {
    return tentPdf(p.x()) * tentPdf(p.y());
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    return Point2f(r * cos(theta), r * sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (p.norm() <= 1) ? INV_PI : 0.0f;
}

Vector3f angleToPoint(float theta, float phi) {
    float sinTheta = sin(theta);
	return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cos(theta));
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float theta = acos(1 - 2 * sample.x());
    float phi = 2 * M_PI * sample.y();
    return angleToPoint(theta, phi);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
	float theta = acos(sample.x());
	float phi = 2 * M_PI * sample.y();
	return angleToPoint(theta, phi);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return v.z() < 0 ? 0 : INV_TWOPI;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float r = sqrt(sample.x());
    float theta = 2 * M_PI * sample.y();
    return Vector3f(r * cos(theta), r * sin(theta), sqrt(1 - sample.x()));
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return v.z() < 0 ? 0 : INV_PI * v.z();
}

Vector3f Warp::squareToBeckmann(const Point2f& sample, float alpha) {
    float phi = 2 * M_PI * sample.y();
    float theta = atan(sqrt(-alpha * alpha * log(sample.x())));
    return angleToPoint(theta, phi);
}

// probability for solid angle instead of single theta
// p(w_h) = D(w_o)cos(theta_h)
// detail see https://zhuanlan.zhihu.com/p/719427294 Section 6.1
float Warp::squareToBeckmannPdf(const Vector3f& m, float alpha) {
    if(m.z() <= 0) return 0;
    float cosTheta = m.z();
    float tan2theta = (m.x() * m.x() + m.y() * m.y()) / (cosTheta * cosTheta);
    float alpha2 = alpha * alpha;
    float cos3theta = cosTheta * cosTheta * cosTheta;
    return INV_PI * exp(-tan2theta / alpha2) / (alpha2 * cos3theta);
}
NORI_NAMESPACE_END
