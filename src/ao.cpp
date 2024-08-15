#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
#define GLOBALAO
#define _ALPHA 0.1

class AoIntegrator : public Integrator {
public:
    AoIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

        // Sampler is using independent sampling
        Point2f sample = sampler->next2D();
        Vector3f sampleDir = its.shFrame.toWorld( Warp::squareToCosineHemisphere(sample) ).normalized();

        Ray3f aoRay(its.p, sampleDir);
        aoRay.mint = 0.001;
#ifdef GLOBALAO
        aoRay.maxt = std::numeric_limits<float>::infinity();
#elif
        aoRay.maxt = _ALPHA;
#endif
        if(scene->rayIntersect(aoRay)) return Color3f(0.0f);

        Normal3f n = its.shFrame.n.cwiseAbs();
        float cosTheta = std::max(0.0f, n.dot(sampleDir));
        return Color3f(INV_PI * cosTheta);
    }

    std::string toString() const {
        return "AoIntegrator[]";
    }
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END