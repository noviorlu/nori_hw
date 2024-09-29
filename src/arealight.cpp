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
		if (rec.wo.z() > 0) {
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

		rec.wi = (rec.refp - rec.p);
		float squareNorm = rec.wi.squaredNorm();
		float norm = sqrt(squareNorm);
		rec.wi /= norm;

		float cosTheta = rec.n.dot(rec.wi);
		float cosThetaRef = rec.refn.dot(-rec.wi);
		if (cosTheta < 0.0f || cosThetaRef < 0.0f) return Color3f(0.0f);

		rec.factor = cosTheta * cosThetaRef / squareNorm;
		rec.radiance = m_radiance;
		rec.pdf = m_sampler->pdf(rec);

		rec.shadowRay = Ray3f(rec.refp, -rec.wi);
		rec.shadowRay.mint = EPSILON;   
		rec.shadowRay.maxt = norm - EPSILON;

		return rec.factor * rec.invpdf * rec.radiance;
	}
	
	float pdf(const EmitterQueryRecord& rec) const override {
		return 0;
	}

	std::string toString() const override {
		return "AreaLight with radiance " + m_radiance.toString();
	}
};
NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END
