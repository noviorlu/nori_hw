#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Result(0.0f), fr(0.0f), Le(0.0f);
        Color3f beta(1.0f, 1.0f, 1.0f);
        float eta = 1.0f;
        Intersection its;
        Ray3f r = ray;
        const BSDF* bsdf;

        float emitterPdf = scene->getPDF();
        float bsdfPdf = 1.0f;
        float w_bsdf = 1.0f;

        for (int depth = 0; depth < 100; depth++) {
            // Path Trace Hit
            if (!scene->rayIntersect(r, its)) break;
            bsdf = its.mesh->getBSDF();

            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(0.99f, beta.maxCoeff());
                if (sampler->next1D() > q) break;
                beta /= q;
            }

            EmitterQueryRecord rec(its.toLocal(-r.d), its.p, its.shFrame.n);
            if (its.mesh->isEmitter()) {
                const auto& emitter = its.mesh->getEmitter();
                Result += w_bsdf * beta * its.mesh->getEmitter()->eval(rec);

                w_bsdf = bsdfPdf / (bsdfPdf+emitterPdf);
            }
            else { // get direct illumination
                Vector3f wo = its.toLocal(-r.d);
                EmitterQueryRecord rec(wo, its.p, its.shFrame.n);

                const Emitter* emitter = scene->SampleLight(rec, sampler);
                Color3f Li = emitter->sample(rec, sampler);

                if (!scene->rayIntersect(rec.shadowRay)) {
                    BSDFQueryRecord bRec(its.toLocal(-rec.wi), wo, ESolidAngle);
                    Color3f f = bsdf->eval(bRec);
                    bsdfPdf = bsdf->pdf(bRec);

                    float w_emitter = emitterPdf / (bsdfPdf+emitterPdf);
                    Result += w_emitter * beta * f * Li;
                }
            }

            BSDFQueryRecord bRec(its.toLocal(-r.d));
            fr = bsdf->sample(bRec, sampler->next2D());
            bsdfPdf = bsdf->pdf(bRec);

            beta *= fr * bRec.eta * bRec.eta;

            r = Ray3f(its.p, its.toWorld(bRec.wo));
        }

        return Result;
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END