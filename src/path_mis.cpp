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
        // emitted
        EmitterQueryRecord lRec(r.o, its.p, its.shFrame.n);
        if (its.mesh->isEmitter()) {
            Result += beta * w_bsdf * its.mesh->getEmitter()->eval(lRec);
        }
        for (int depth = 0; depth < 10; depth++) {
            float pdf_mat = 0.0f, pdf_em = 0.0f;
            Color3f MC_em = Color3f(0.0f), MC_bsdf = Color3f(0.0f);

            const Emitter* light = scene->SampleLight(lRec, sampler);
            Color3f Li = light->sample(lRec, sampler);
            pdf_em = light->pdf(lRec);
            if (!scene->rayIntersect(lRec.shadowRay)) {
                BSDFQueryRecord bRec(its.toLocal(-r.d), its.toLocal(lRec.wi), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bRec);

                MC_em = Li * f;
            }
            //if (depth >= 3) {
            //    float q = std::min(0.99f, beta.maxCoeff() * eta * eta);
            //    if (sampler->next1D() > q || q == 0.0f) break;
            //    beta /= q;
            //}

            //BSDF
            BSDFQueryRecord bRec(its.shFrame.toLocal(-r.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            pdf_mat = its.mesh->getBSDF()->pdf(bRec);

            r = Ray3f(its.p, its.toWorld(bRec.wo));
            if (!scene->rayIntersect(r, its)) break;

            pdf_mat *= its.shFrame.n.dot(-r.d) / (its.p - r.o).squaredNorm();
            if (pdf_mat < 0) pdf_mat = 0.0f;
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord lRec = EmitterQueryRecord(r.o, its.p, its.shFrame.n);
                MC_bsdf = f * its.mesh->getEmitter()->eval(lRec);
            }

            if(pdf_em + pdf_mat == 0.0f) break;
            float w_em = pdf_em / (pdf_em + pdf_mat);
            float w_bsdf = pdf_mat / (pdf_em + pdf_mat);
            Result += beta * (w_bsdf * MC_bsdf + w_em * MC_em);

            eta *= bRec.eta;
            beta *= f;
        }

        return Result;
    }



    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END