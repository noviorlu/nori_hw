#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:
    /* parameters */
    Point3f m_lightPos;
    Color3f m_lightIntensity;
public:
    SimpleIntegrator(const PropertyList &props) {
        /* No parameters this time */
        m_lightPos = props.getPoint("position");
        m_lightIntensity = props.getColor("energy");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute
           value of the shading normal as a color */
        Normal3f n = its.shFrame.n.cwiseAbs();
        return Color3f(n.x(), n.y(), n.z());
    }

    std::string toString() const {
        return "SimpleIntegrator[]";
    }
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END