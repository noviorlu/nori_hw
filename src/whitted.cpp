#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
NORI_NAMESPACE_BEGIN
#define GLOBALAO
#define _ALPHA 0.1

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);
        
        EmitterQueryRecord rec(its.p, its.shFrame.n);
        Color3f Le(0.0);
        if (its.mesh->isEmitter()) Le = its.mesh->getEmitter()->eval(rec);

        Color3f Li = scene->SampleLight(rec, sampler)->sample(rec, sampler);
        if (scene->rayIntersect(rec.shadowRay)) return Le;

        BSDFQueryRecord bRec(its.toLocal(-rec.wi), its.toLocal(-ray.d), ESolidAngle);
        Color3f f = its.mesh->getBSDF()->eval(bRec);

        return Le + Li * f;
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END