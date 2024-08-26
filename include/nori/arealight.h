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
		return (rec.n.dot(rec.wi) < 0.0f) ? rec.factor * m_radiance : 0.0f;
	}

	Color3f sample(EmitterQueryRecord &rec, Sampler* sampler) const override{
		if (m_sampler == nullptr) {
			cout << "AreaLight::sample: m_sampler is nullptr" << endl;
			return Color3f(0.0f);
		}

		m_sampler->sample(rec, sampler);
		rec.wi = rec.ref - rec.p;
		float dist = rec.wi.norm();
		rec.wi.normalize();
		rec.factor = std::max(0.0f, rec.n.dot(-rec.wi)) / (dist * dist) * m_sampler->invpdf(rec);
		rec.radiance = m_radiance;

		rec.shadowRay = Ray3f(rec.ref, rec.wi);
		rec.shadowRay.mint = EPSILON;
		rec.shadowRay.maxt = dist - EPSILON;

		return rec.factor * rec.radiance;
	}
	
	// transform pdf from solid angle to area
	float pdf(const EmitterQueryRecord& rec) const override {
		return m_sampler.pdf();
	}
};
NORI_REGISTER_CLASS(AreaLight, "area")
NORI_NAMESPACE_END
