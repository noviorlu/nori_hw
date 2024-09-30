#pragma once

#include <nori/emitter.h>
#include <nori/sampler.h>
NORI_NAMESPACE_BEGIN

#define EPSILON 0.0001f
/**
 * Each triangle of a mesh that is marked as an area light uniformly emits 
 * radiance towards all directions above its surface
 */
class AreaLight : public Emitter {
public:
	Color3f m_radiance;
public:
	AreaLight(const PropertyList &propList) {
		m_radiance = propList.getColor("radiance", Color3f(1.0f));
	}

	Color3f getRadiance() const {
		return m_radiance;
	}

	Color3f eval(const EmitterQueryRecord& rec) const override {
		if (rec.wi.dot(rec.refn) > 0) {
			return getRadiance();
		}
		else {
			return Color3f(0.0f);
		}
	}

	Color3f sample(EmitterQueryRecord &rec, Sampler* sampler) const override{
		if (m_sampler == nullptr) {
			cout << "AreaLight::sample: m_sampler is nullptr" << endl;
			return Color3f(0.0f);
		}
		m_sampler->sample(rec, sampler);

		float squareNorm = (rec.p - rec.refp).squaredNorm();
		float cosTheta = rec.n.dot(-rec.wo);
		float cosThetaRef = rec.refn.dot(rec.wo);
		if (cosTheta < 0.0f || cosThetaRef < 0.0f) return Color3f(0.0f);

		rec.factor = cosTheta * cosThetaRef / squareNorm;
		rec.radiance = m_radiance;

		rec.shadowRay = Ray3f(rec.refp, rec.wo);
		rec.shadowRay.mint = EPSILON;   
		rec.shadowRay.maxt = sqrt(squareNorm) - EPSILON;

		return rec.factor * rec.invpdf * rec.radiance;
	}
	
	float pdf(const EmitterQueryRecord& rec) const override {
		if (rec.factor == -1.0f) {
			// hit Emitter, current hitpoint is Emitter thus
			float cosTheta = rec.refn.dot(rec.wi);
			if (cosTheta > 0.0f) {
				return (rec.viewp - rec.refp).squaredNorm() / cosTheta;
			}
		}
		else {
			float cosTheta = rec.n.dot(-rec.wo);
			if (cosTheta > 0.0f) {
				return (rec.p - rec.refp).squaredNorm() / cosTheta;
			}
		}
		
		return 0.0f;
	}

	std::string toString() const override {
		return "AreaLight with radiance " + m_radiance.toString();
	}
};
NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END
