#pragma once

#include <nori/object.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN
struct EmitterQueryRecord {
    Point3f ref;
    Vector3f wi;
    Point3f p;
    Normal3f n;
    Color3f radiance;
    Ray3f shadowRay;
    // here is a mix of Geometry Term and pdf combined
    float factor;
};

class IEmitterSampler {
public:
    /// set only p, n, pdf, rest handled by the emitter
    virtual void preprocess() = 0;
    virtual Color3f sample(EmitterQueryRecord& rec, Sampler* sampler) const = 0;
    virtual float pdf(EmitterQueryRecord& rec) const = 0;
    virtual float invpdf(EmitterQueryRecord& rec) const = 0;
};

NORI_NAMESPACE_END
