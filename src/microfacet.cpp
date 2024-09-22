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

    // D(wh) = pdf(wh) * cos(theta_h)
    float BeckmannD(const Vector3f& wh) const {
		return Warp::squareToBeckmannPdf(wh, m_alpha) * Frame::cosTheta(wh);
	}

    float BeckmannG1(const Vector3f& wv, const Vector3f& wh) const {
        if (wv.dot(wh) / Frame::cosTheta(wv) <= 0) return 0;

        // b = (atanv)^-1
        float b = 1 / (m_alpha * sqrt(1 - wv.z() * wv.z()));
        float b2 = b * b;
        return b < 1.6f ? (3.535f * b + 2.181f * b2) / (1 + 2.276f * b + 2.577f * b2) : 1.0f;
    }

    float BeckmannG(const Vector3f& wi, const Vector3f& wo, const Vector3f& wh) const {
		return BeckmannG1(wi, wh) * BeckmannG1(wo, wh);
	}

    float Fresnel(const Vector3f& wi, const Vector3f& wh) const {
        return fresnel(wi.dot(wh), m_extIOR, m_intIOR);
    }

    float BeckmannMicrofacet(const Vector3f& wi, const Vector3f& wo) const {
		Vector3f wh = (wi + wo).normalized();
		return BeckmannD(wh) * BeckmannG(wi, wo, wh) * Fresnel(wi, wh) / (4 * Frame::cosTheta(wi) * Frame::cosTheta(wo));
	}

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
    	// lerp between diffuse and microfacet
        return m_kd * INV_PI + m_ks * BeckmannMicrofacet(bRec.wi, bRec.wo);
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
    	return (1-m_ks) * Warp::squareToCosineHemispherePdf(bRec.wo) 
            + m_ks * Warp::squareToBeckmannPdf(wh, m_alpha) / 4 / std::abs(wh.dot(bRec.wo));
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        if (Frame::cosTheta(bRec.wi) <= 0) return Color3f(0.0f);
        
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        if (_sample.x() > m_ks) {
            Point2f sample((_sample.x() - m_ks) / (1.f - m_ks), _sample.y());
			bRec.wo = Warp::squareToCosineHemisphere(_sample);
        }
        else {
            Point2f sample(_sample.x() / m_ks, _sample.y());
			Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
			bRec.wo = 2 * wh.dot(bRec.wi) * wh - bRec.wi;
		}
        if (Frame::cosTheta(bRec.wo) < 0.f) { return Color3f(0.0f); }

        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
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
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
