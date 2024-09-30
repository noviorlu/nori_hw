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

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
        Color3f Result(0.0f);
        Color3f beta(1.0f, 1.0f, 1.0f);
        float eta = 1.0f;
        
        Intersection its;
        Ray3f r = ray;
        
        float w_bsdf = 1.0f;

        if (!scene->rayIntersect(r, its)) return Result;
        for (int depth = 0; depth < 100; depth++) {
            // emitted
            EmitterQueryRecord lRec(r.o, its.p, its.shFrame.n);
            if (its.mesh->isEmitter()) {
                Result += beta * w_bsdf * its.mesh->getEmitter()->eval(lRec);
            }

            const Emitter* light = scene->SampleLight(lRec, sampler);
            Color3f Li = light->sample(lRec, sampler);
            float pdf_em = scene->getPDF() * light->pdf(lRec);
            if (!scene->rayIntersect(lRec.shadowRay)) {
                float cosTheta = std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec.wi)));

                BSDFQueryRecord bRec(its.toLocal(-r.d), its.toLocal(lRec.wi), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bRec);
                float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                float w_ems = pdf_mat + pdf_em > 0.f ? pdf_em / (pdf_mat + pdf_em) : pdf_em;

                Result += Li * f * w_ems * beta;
            }
            if (depth >= 3) {
                float q = std::min(0.99f, beta.maxCoeff() * eta * eta);
                if (sampler->next1D() > q || q == 0.0f) break;
                beta /= q;
            }

            //BSDF
            BSDFQueryRecord bRec(its.shFrame.toLocal(-r.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

            eta *= bRec.eta;
            beta *= f;

            r = Ray3f(its.p, its.toWorld(bRec.wo));
            if (!scene->rayIntersect(r, its)) break;

            if (its.mesh->isEmitter()) {
                EmitterQueryRecord lRec = EmitterQueryRecord(r.o, its.p, its.shFrame.n);
                float pdf_em = scene->getPDF() * its.mesh->getEmitter()->pdf(lRec);
                w_bsdf = pdf_mat + pdf_em > 0.f ? pdf_mat / (pdf_mat + pdf_em) : pdf_mat;
            }
        }

        return Result;
    }



    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END