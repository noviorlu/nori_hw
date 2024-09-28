#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Result(0.0f), fr(0.0f), Le(0.0f);
        Color3f beta(1.0f, 1.0f, 1.0f);

        Intersection its;
        
        const BSDF* bsdf;
        Ray3f r = ray;

        for (int depth = 0; depth < 10; depth++) {
            // Path Trace Hit
            if (!scene->rayIntersect(r, its)) break;
            
            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(0.99f, beta.maxCoeff());
                if (sampler->next1D() > q) break;
                beta /= q;
            }

            // Emitter Hit
            EmitterQueryRecord rec(its.p, its.shFrame.n);
            if (its.mesh->isEmitter()) Le = its.mesh->getEmitter()->eval(rec);
            else Le = Color3f(0.0f);

            Result += beta * Le;

            BSDFQueryRecord bRec(its.toLocal(-r.d));
            bsdf = its.mesh->getBSDF();
            fr = bsdf->sample(bRec, sampler->next2D());
            
            beta *= fr * Frame::cosTheta(bRec.wo);
            r = Ray3f(its.p, its.toWorld(bRec.wo));
        }

        return Result;
        //Intersection its;
        //if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

        //const BSDF* bsdf = its.mesh->getBSDF();
        //if (bsdf->isDiffuse()) {
        //    EmitterQueryRecord rec(its.p, its.shFrame.n);
        //    Color3f Le(0.0);
        //    if (its.mesh->isEmitter()) Le = its.mesh->getEmitter()->eval(rec);

        //    Color3f Li = scene->SampleLight(rec, sampler)->sample(rec, sampler);
        //    if (scene->rayIntersect(rec.shadowRay)) return Le;

        //    BSDFQueryRecord bRec(its.toLocal(-rec.wi), its.toLocal(-ray.d), ESolidAngle);
        //    Color3f f = bsdf->eval(bRec);
        //    return Le + Li * f;
        //}
        //else {
        //    BSDFQueryRecord bRec(its.toLocal(-ray.d));
        //    Color3f f = bsdf->sample(bRec, sampler->next2D());

        //    if(f.x() == 0.0f || sampler->next1D() > 0.95) return Color3f(0.0f);
        //    return f * Li(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo))) / 0.95;
        //}
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END