#pragma once

#include <nori/object.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN
#define EPSILON 0.0001f

struct EmitterQueryRecord {
    EmitterQueryRecord() {}
    EmitterQueryRecord(const Point3f& ref, const Normal3f& refn) {
        this->ref = ref;
        this->refn = refn;
    }

    Point3f ref;
    Normal3f refn;

    Vector3f wi;
    Point3f p;
    Normal3f n;
    Color3f radiance;
    Ray3f shadowRay;

    // here is a mix of Geometry Term and pdf combined
    long double pdf = 1.0;
    long double invpdf = 1.0;
    float factor = 0;
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
