#pragma once

#include <nori/object.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN
#define EPSILON 0.0001f

struct EmitterQueryRecord {
    EmitterQueryRecord() {}
    EmitterQueryRecord(const Point3f& viewp, const Point3f& refp, const Normal3f& refn) {
        this->refp = refp;
        this->refn = refn;
        this->viewp = viewp;
        this->wi = (viewp - refp).normalized(); // dir from refp to viewp
    }

    EmitterQueryRecord(
        const Point3f& viewp, 
        const Point3f& refp, const Normal3f& refn, 
        const Point3f& p, const Normal3f& n
    ) {
        this->refp = refp;
		this->refn = refn;
		this->viewp = viewp;
		this->wi = (viewp - refp).normalized(); // dir from refp to viewp
		this->p = p;
		this->n = n;
		this->wo = (p - refp).normalized(); // dir from refp to p
    }

    // Info from the ray Hit point
    Point3f viewp; // view point
    Point3f refp; // current Hit point 
    Normal3f refn; // normal at the Hit point (global)
    Vector3f wi; // direction from the Hit point to view (global)

    // Info generated during sampling emitter
    Point3f p; // sampled point on the emitter
    Normal3f n; // normal at the sampled point on the emitter
    Vector3f wo; // direction from Hit point to the sampled emitter (global)

    Color3f radiance;
    Ray3f shadowRay;

    // Geometry Term and pdf combined
    long double pdf;
    long double invpdf;
    float factor = -1.0f;
};

class IEmitterSampler {
public:
    /// set only p, n, pdf, rest handled by the emitter
    virtual void preprocess() = 0;
    virtual Color3f sample(EmitterQueryRecord& rec, Sampler* sampler) const = 0;
    virtual float pdf(const EmitterQueryRecord& rec) const = 0;
    virtual float sum(const EmitterQueryRecord& rec) const = 0;
};

NORI_NAMESPACE_END
