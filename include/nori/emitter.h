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

#pragma once
#include <nori/emittersampler.h>
NORI_NAMESPACE_BEGIN
/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:
    IEmitterSampler* m_sampler;
public:
    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }

    void preprocess() { if(m_sampler) m_sampler->preprocess(); }

    virtual Color3f sample(EmitterQueryRecord& rec, Sampler* sampler) const = 0;

    virtual Color3f eval(const EmitterQueryRecord& rec) const = 0;

    virtual float pdf(const EmitterQueryRecord& rec) const { return 1.0f; }

    virtual float sum(const EmitterQueryRecord& rec) const { return m_sampler->sum(rec); }
};

NORI_NAMESPACE_END
