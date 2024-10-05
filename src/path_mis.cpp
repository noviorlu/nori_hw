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
        
        // Self Emission, first hit light
        if (!scene->rayIntersect(r, its)) return Result;
        if (its.mesh->isEmitter()) {
			EmitterQueryRecord lRec(r.o, its.p, its.shFrame.n);
			Result += its.mesh->getEmitter()->eval(lRec);
		}

        bool stopFlag = false;
        for (int depth = 0; depth < 10 && !stopFlag; depth++) {
            double w_bsdf = 1.0f, w_emitter = 1.0f;
            double pdf_bsdf = 0.0f, pdf_emitter = 0.0f;

            Color3f MC_emitter(0.0f), MC_bsdf(0.0f);
            Color3f deltaB(1.0f);
            float deltaEta = 1.0f;

            { // Light Sampling
                EmitterQueryRecord lRec(r.o, its.p, its.shFrame.n);
                const Emitter* light = scene->SampleLight(lRec, sampler);
                MC_emitter = light->sample(lRec, sampler); // Le * G / pdf

                if (!scene->rayIntersect(lRec.shadowRay)) {
                    BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(lRec.wo), ESolidAngle);
                    Color3f f = its.mesh->getBSDF()->eval(bRec);

                    MC_emitter *= f * scene->m_lights.size();

                    // divide by the scene pdf
                    pdf_emitter = lRec.pdf / scene->m_lights.size(); // in 1/dA
                    pdf_emitter *= (lRec.p - lRec.refp).squaredNorm() / lRec.n.dot(-lRec.wo); // from Area to solidAngle
                }
                else {
                    MC_emitter = Color3f(0.0f);
                }
            }
            
            { // BSDF Sampling
                BSDFQueryRecord bRec(its.shFrame.toLocal(-r.d));
                const BSDF* bsdf = its.mesh->getBSDF();
                // This two line can be optimized since pdf is recalculating if using NDF
                // deltaB = fr * cos(theta) / pdf
                deltaB = bsdf->sample(bRec, sampler->next2D());
                pdf_bsdf = bsdf->pdf(bRec); // whether NDF or VNDF

                // deltaEta = bRec.eta;

                r = Ray3f(its.p, its.toWorld(bRec.wo));
                if (!scene->rayIntersect(r, its)) stopFlag = true;

                if (!stopFlag && its.mesh->isEmitter()) {
                    EmitterQueryRecord lRec(r.o, its.p, its.shFrame.n);
                    Color3f Li = its.mesh->getEmitter()->eval(lRec); // if not backface return radiance
                    MC_bsdf = Li * deltaB; // Le * G / pdf
                
                    //if(Li.maxCoeff() == 0.0f) pdf_bsdf = 0.0f;
                    //else pdf_bsdf *= its.shFrame.n.dot(-r.d.normalized()) / (r.o - its.p).squaredNorm(); // from Area to solidAngle
                }
            }

            // Balance Huristic
            if (pdf_emitter + pdf_bsdf != 0.0f) {
                //Result += beta * MC_bsdf;
                //Result += beta * MC_emitter;
                w_bsdf = pdf_bsdf / (pdf_bsdf + pdf_emitter);
                w_emitter = pdf_emitter / (pdf_bsdf + pdf_emitter);
                Result += beta * (w_bsdf * MC_bsdf + w_emitter * MC_emitter);
            }
            
            beta *= deltaB;
            eta *= deltaEta;
            // ray already updated in BSDF Sampling so no need update here
            // goes into the next iteration
            //if (depth >= 3) {
            //    float q = std::min(0.99f, beta.maxCoeff() * eta * eta);
            //    if (sampler->next1D() > q || q == 0.0f) break;
            //    beta /= (1 - q);
            //}
        }

        return Result;
    }



    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END