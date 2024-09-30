#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
NORI_NAMESPACE_BEGIN

class PathEmsIntegrator : public Integrator {
public:
    PathEmsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Result(0.0f), fr(0.0f), Le(0.0f);
        Color3f beta(1.0f, 1.0f, 1.0f);
        float eta = 1.0f;
        Intersection its;
        Ray3f r = ray;
        const BSDF* bsdf;
        int isDelta = 1;

        for (int depth = 0; depth < 100; depth++) {
            // Path Trace Hit
            if (!scene->rayIntersect(r, its)) break;
            bsdf = its.mesh->getBSDF();

            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(0.99f, beta.maxCoeff() * eta * eta);
                if (sampler->next1D() > q) break;
                beta /= q;
            }

            if (bsdf->isDiffuse()) {
                EmitterQueryRecord rec(its.toLocal(-r.d), its.p, its.shFrame.n);
                if (its.mesh->isEmitter()) {
                    Result += beta * its.mesh->getEmitter()->eval(rec) * isDelta;
                }
                else { // direct illumination
                    Vector3f wo = its.toLocal(-r.d);
                    EmitterQueryRecord rec(wo, its.p, its.shFrame.n);

                    Color3f Li = scene->SampleLight(rec, sampler)->sample(rec, sampler);
                    if (!scene->rayIntersect(rec.shadowRay)) {
                        BSDFQueryRecord bRec(its.toLocal(-rec.wi), wo, ESolidAngle);
                        Color3f f = bsdf->eval(bRec);
                        //beta *= 0.5;
                        Result += beta * f * Li;
                    }
                    isDelta = 0;
                }
            }
            else isDelta = 1;

            BSDFQueryRecord bRec(its.toLocal(-r.d));
            fr = bsdf->sample(bRec, sampler->next2D());

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

NORI_REGISTER_CLASS(PathEmsIntegrator, "path_ems");
NORI_NAMESPACE_END