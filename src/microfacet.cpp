/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    float BeckmannG1(const Vector3f& wv, const Vector3f& wh) const {
        if (wv.dot(wh) / Frame::cosTheta(wv) <= 0) return 0;

        // b = (atanv)^-1
        float b = 1 / (m_alpha * sqrt(1 - wv.z() * wv.z()));
        float b2 = b * b;
        return b < 1.6f ? (3.535f * b + 2.181f * b2) / (1 + 2.276f * b + 2.577f * b2) : 1.0f;
    }

    float BeckmannMicrofacet(const Vector3f& wi, const Vector3f& wo) const {
		Vector3f wh = (wi + wo).normalized();
        float f = fresnel(wi.dot(wh), m_extIOR, m_intIOR);
        float d = BeckmannD(wh);
        float g = BeckmannG1(wi, wh) * BeckmannG1(wo, wh);
		return f * d * g / (4 * Frame::cosTheta(wi) * Frame::cosTheta(wo));
	}

    float BeckmannD(const Vector3f& wh) const {
		return Warp::squareToBeckmannPdf(wh, m_alpha) / Frame::cosTheta(wh);
	}

    float BeckmannDh(const Vector3f& wh) const {
        return  BeckmannG1(Vector3f(0, 0, 1), wh) / Frame::cosTheta(wh) * BeckmannD(wh) * std::max(0.0f, wh.z());
	}

    float GGXG1(const Vector3f& wv, const Vector3f& wh) const {
	    if (wv.dot(wh) / Frame::cosTheta(wv) <= 0) return 0;

        float cosV = Frame::cosTheta(wv);
        float cosV2 = cosV * cosV;
        float tanV2 = 1 / cosV2 - 1;

        return 2 / (1 + sqrt(1 + m_alpha * m_alpha * tanV2));
    }

    float GGXMicrofacet(const Vector3f& wi, const Vector3f& wo) const {
        Vector3f wh = (wi + wo).normalized();
        float f = fresnel(wi.dot(wh), m_extIOR, m_intIOR);
        float d = GGXD(wh);
        float g = GGXG1(wi, wh) * GGXG1(wo, wh);
        return f * d * g / (4 * Frame::cosTheta(wi) * Frame::cosTheta(wo));
    }

    float GGXD(const Vector3f& wh) const {
		return Warp::squareToGGXPdf(wh, m_alpha) / Frame::cosTheta(wh);
	}

    float GGXDh(const Vector3f& wh) const {
        return GGXG1(Vector3f(0, 0, 1), wh) / Frame::cosTheta(wh) * GGXD(wh) * std::max(0.0f, wh.z());
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        if(isGGX) return m_kd * INV_PI + m_ks * GGXMicrofacet(bRec.wi, bRec.wo);
        else return m_kd * INV_PI + m_ks * BeckmannMicrofacet(bRec.wi, bRec.wo);
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        if(Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0) return 0.0f;
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float jacobian = 1 / (4.0f * abs(wh.dot(bRec.wo)));
        
        float kspdf;
        if (isVNDF) {
            if(isGGX) kspdf = m_ks * GGXDh(wh) * jacobian;
			else kspdf = m_ks * BeckmannDh(wh) * jacobian;
        }
        else {
            if (isGGX) kspdf = m_ks * Warp::squareToGGXPdf(wh, m_alpha) * jacobian;
            else kspdf = m_ks * Warp::squareToBeckmannPdf(wh, m_alpha) * jacobian;
        }

        float kdpdf = (1 - m_ks) * Warp::squareToCosineHemispherePdf(bRec.wo);
        return kdpdf + kspdf;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        if (isVNDF) {
            // sample SphericalCap
            Vector3f Vh = -bRec.wi;
            Vh.z() *= -1;

            float phi = 2.0f * M_PI * _sample.y();
            float z = (1.0f - _sample.x()) * (1.0f + Vh.z()) - Vh.z();
            float sinTheta = sqrt(clamp(1.0f - z * z, 0.0f, 1.0f));
            float x = sinTheta * cos(phi);
            float y = sinTheta * sin(phi);
            Vector3f c(x, y, z);
            bRec.wo = (c + Vh).normalized();
            if(bRec.wo.z() * bRec.wi.z() <= 0) return Color3f(0.0f);

            // VNDF 2018
            //const Vector3f& Vh = Vector3f(
            //    -bRec.wi.x(),
            //    -bRec.wi.y(),
            //    bRec.wi.z()
            //);
            //float lensq = Vh.x() * Vh.x() + Vh.y() * Vh.y();
            //Vector3f T1 = lensq > 0 ? Vector3f(-Vh.y(), Vh.x(), 0) / sqrt(lensq) : Vector3f(1, 0, 0);
            //Vector3f T2 = Vh.cross(T1);
            //float r = sqrt(_sample.x());
            //float phi = 2.0 * M_PI * _sample.y();
            //float t1 = r * cos(phi);
            //float t2 = r * sin(phi);
            //float s = 0.5 * (1.0 + Vh.z());
            //t2 = (1.0 - s) * sqrt(1.0 - t1 * t1) + s * t2;
            //bRec.wo = t1 * T1 + t2 * T2 + sqrt(std::max(0.0, 1.0 - t1 * t1 - t2 * t2)) * Vh;
            
            return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
            //Vector3f wh = (bRec.wi + bRec.wo).normalized();
            //return GGXG1(bRec.wo, wh) * fresnel(bRec.wi.dot(wh), m_extIOR, m_intIOR);
        }
        else {

            if (Frame::cosTheta(bRec.wi) <= 0) return Color3f(0.0f);

            if (_sample.x() > m_ks) {
                Point2f sample((_sample.x() - m_ks) / (1.f - m_ks), _sample.y());
                bRec.wo = Warp::squareToCosineHemisphere(sample);
            }
            else {
                Point2f sample(_sample.x() / m_ks, _sample.y());
                Vector3f wh;
                if (isGGX) wh = Warp::squareToGGX(sample, m_alpha);
                else wh = Warp::squareToBeckmann(sample, m_alpha);
                bRec.wo = 2 * wh.dot(bRec.wi) * wh - bRec.wi;
            }
            if (Frame::cosTheta(bRec.wo) < 0.f) return Color3f(0.0f);
            return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        }
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    bool isGGX = true;
    bool isVNDF = false;
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
