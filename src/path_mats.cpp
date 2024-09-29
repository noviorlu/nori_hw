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
        float eta = 1.0f;
        Intersection its;
        Ray3f r = ray;
        const BSDF* bsdf;

        for (int depth = 0; depth < 100; depth++) {
            // Path Trace Hit
            if (!scene->rayIntersect(r, its)) break;
            
            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(0.99f, beta.maxCoeff() * eta * eta);
                if (sampler->next1D() > q) break;
                beta /= q;
            }

            // Emitter Hit
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord rec(its.toLocal(-r.d), its.p, its.shFrame.n);
                Result += beta * its.mesh->getEmitter()->eval(rec);
                break;
            }

            BSDFQueryRecord bRec(its.toLocal(-r.d));
            fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            
            eta *= bRec.eta;
            beta *= fr/* * std::abs(Frame::cosTheta(bRec.wo))*/;  // ignored cosTheta here since it's not devided in the pdf
            
            r = Ray3f(its.p, its.toWorld(bRec.wo));
        }

        return Result;
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END