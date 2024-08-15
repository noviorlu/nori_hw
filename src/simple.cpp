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
        Intersection its;
        if (!scene->rayIntersect(ray, its)) return Color3f(0.0f);

        Point3f x = its.p, p = m_lightPos;
        Vector3f l = (p - x).normalized();
        if(scene->rayIntersect(Ray3f(x + 0.001 * l, l))) return Color3f(0.0f);
        
        float r2 = (p - x).squaredNorm();
        Normal3f n = its.shFrame.n.cwiseAbs();
        float cosTheta = std::max(0.0f, n.dot(l));
        float k = 0.25 * INV_PI * INV_PI;

        return m_lightIntensity * k * cosTheta / r2;
    }

    std::string toString() const {
        return "SimpleIntegrator[Position: " + m_lightPos.toString() + ", Energy: " + m_lightIntensity.toString() + "]";
    }
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END