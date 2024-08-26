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

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() { }

Mesh::~Mesh() {
    delete m_bsdf;
    delete m_emitter;
}

void Mesh::activate() {
    if (isActivate) return;

    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }
    int triCount = this->getTriangleCount();
    if (m_emitter && triCount) {
		/* If emitter assigned, init sampler */
        m_dpdf.reserve(triCount);
        for (int i = 0; i < triCount; i++)
            m_dpdf.append(this->surfaceArea(i));
        m_dpdf.normalize();
	}

    isActivate = true;
}

float Mesh::surfaceArea(uint32_t index) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

void Mesh::getTriangle(int index, Point3f& v0, Point3f& v1, Point3f& v2) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    v0 = m_V.col(i0); v1 = m_V.col(i1); v2 = m_V.col(i2);
}

void Mesh::getTriangleIdx(int index, uint32_t& i0, uint32_t& i1, uint32_t& i2) const
{
    i0 = m_F(0, index); i1 = m_F(1, index); i2 = m_F(2, index);
}

bool Mesh::rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

BoundingBox3f Mesh::getBoundingBox(uint32_t index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(uint32_t index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}

/// <summary>
/// Uniformly sample on the mesh sufrace
/// </summary>
/// <param name="rec"> Result Recorder </param>
/// <param name="sampler"> sampler it's using </param>
/// <returns></returns>
Color3f Mesh::sample(EmitterQueryRecord& queryRec, Sampler* sampler) const
{
    float triSample = sampler->next1D();
    Point2f bycentricSample = sampler->next2D();

    float tmp = std::sqrt(bycentricSample.x());
    float alpha = 1 - tmp;
    float beta = tmp * bycentricSample.y();
    float gamma = 1 - alpha - beta;

    int sampleIdx = m_dpdf.sample(triSample);

    uint32_t i0, i1, i2;
    getTriangleIdx(sampleIdx, i0, i1, i2);
    Point3f v0 = m_V.col(i0), v1 = m_V.col(i1), v2 = m_V.col(i2);

    queryRec.p = alpha * v0 + beta * v1 + gamma * v2;
    
    // solve normal, if exist barycentric, if not exist counterclockwise
    if(m_N.cols() > 0)
	{
        Point3f n0 = m_N.col(i0), n1 = m_N.col(i1), n2 = m_N.col(i2);
		queryRec.n = alpha * n0 + beta * n1 + gamma * n2;
	}
	else
	{
        queryRec.n = (v1 - v0).cross(v2 - v0).normalized();
	}

    return Color3f(0.0f);
}

float Mesh::pdf(EmitterQueryRecord& rec) const
{
    return m_dpdf.getNormalization();
}

float Mesh::invpdf(EmitterQueryRecord& rec) const
{
    return m_dpdf.getSum();
}

void Mesh::addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Mesh: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter: {
                Emitter *emitter = static_cast<Emitter *>(obj);
                if (m_emitter)
                    throw NoriException(
                        "Mesh: tried to register multiple Emitter instances!");
                m_emitter = emitter;
                emitter->m_sampler = (IEmitterSampler*)this;
            }
            break;

        default:
            throw NoriException("Mesh::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null")
    );
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
