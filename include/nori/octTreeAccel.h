#pragma once

#include <nori/bbox.h>
#include <nori/mesh.h>
#include <nori/accel.h>

#include <iostream>
#include <stack>
#include <unordered_map>
NORI_NAMESPACE_BEGIN
class Element{
public:
    enum ElementType{
        TRIANGLEIDX
    };

    Element(ElementType type){
        m_type = type;
    }

    virtual bool inBBox(const BoundingBox3f& bbox) const { return false; }
    virtual bool rayIntersect(Ray3f& ray, Intersection& its) const { return false; }
public:
    ElementType m_type;
};

class Triangle : public Element{
public:
    Triangle(Mesh* mesh, int i) : Element(TRIANGLEIDX){
        m_mesh = mesh;
        idx = i;
        m_mesh->getTriangleIdx(i, m_i0, m_i1, m_i2);
        //m_mesh->getTriangle(i, m_p0, m_p1, m_p2);
    }
    
    bool inBBox(const BoundingBox3f& bbox)const override {
        const auto& V = m_mesh->getVertexPositions();
        const Point3f p0 = V.col(m_i0), p1 = V.col(m_i1), p2 = V.col(m_i2);
        if (bbox.contains(p0) && bbox.contains(p1) && bbox.contains(p2)) {
			return true;
		}
        //if (bbox.contains(m_p0) && bbox.contains(m_p1) && bbox.contains(m_p2)) {
        //    return true;
        //}
        return false;
    }

    bool rayIntersect(Ray3f& ray, Intersection& its) const override {
        float u, v, t;
        const auto& V = m_mesh->getVertexPositions();
        const Point3f p0 = V.col(m_i0), p1 = V.col(m_i1), p2 = V.col(m_i2);
        
        /* Find vectors for two edges sharing v[0] */
        Vector3f edge1 = p1 - p0, edge2 = p2 - p0;
        //Vector3f edge1 = m_p1 - m_p0, edge2 = m_p2 - m_p0;

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

        bool rayHit = (t >= ray.mint && t <= ray.maxt);
        if (rayHit) {
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
        }
        return rayHit;
    }

public:
    //Point3f m_p0;
    //Point3f m_p1;
    //Point3f m_p2;

    uint32_t m_i0;
    uint32_t m_i1;
    uint32_t m_i2;
    Mesh* m_mesh;
    int idx;
};

#define MAX_DEPTH 10
#define MAX_WIDTH 4
#define RELASE_CONST 0.1
class OctTreeNode{
public:
    OctTreeNode(const BoundingBox3f& bbox) {
		m_bbox = bbox;
        for (int i = 0; i < 8; i++) {
			m_children[i] = nullptr;
		}
	}
    ~OctTreeNode() {
        for(int i = 0; i < 8; i++) {
            delete m_children[i];
        }

        for(Element* element : m_elements) {
			delete element;
		}
    }

    void print(int level) {
        for (int i = 0; i < level; i++) {std::cout << "  ";}
        std::cout << "level: " << level << "\n";
        for (int i = 0; i < level; i++) { std::cout << "  "; }
        std::cout << "bbox: " << m_bbox.min.toString() << m_bbox.max.toString() << "\n";
        for (int i = 0; i < level; i++) { std::cout << "  "; }
        std::cout << "elements Num : " << m_elements.size() << std::endl;
        for (int i = 0; i < level; i++) { std::cout << "  "; }
        // print elements
        for (Element* element : m_elements) {
            switch (element->m_type) {
				case Element::TRIANGLEIDX:
					std::cout << "; triangle index: " << ((Triangle*)element)->idx;
					break;
				default:
					break;
			}
        }
        std::cout << std::endl;

        for (int i = 0; i < 8; i++) {
            if (m_children[i] != nullptr) {
				m_children[i]->print(level + 1);
			}
		}
    }
    static void validate(OctTreeNode* root, int size) {
        std::cout << "Check Size: " << size << std::endl;
        
        bool shouldExit = false;
        std::vector<bool> flags(size, false);

        std::stack<OctTreeNode*> m_stack;
        m_stack.push(root);
        while (!m_stack.empty()) {
            OctTreeNode* node = m_stack.top();
			m_stack.pop();
            for (Element* element : node->m_elements) {
                if (element->m_type == Element::TRIANGLEIDX) {
					int idx = ((Triangle*)element)->idx;
                    if (flags[idx]) {
						std::cout << "Multiple Existance error: " << idx << std::endl;
                        shouldExit = true;
					}
					flags[idx] = true;
				}
			}

            for (int i = 0; i < 8; i++) {
                if (node->m_children[i] != nullptr) {
					m_stack.push(node->m_children[i]);
				}
			}
        }

        for (int i = 0; i < size; i++) {
			if (!flags[i]) { 
                std::cout << "Triangle Not Load error: " << i << std::endl;
                shouldExit = true;
            }
        }

        if (shouldExit) { exit(-1); }
    }

    Element* rayIntersect(Ray3f& ray, Intersection& its) const;

    static OctTreeNode* build(const BoundingBox3f& bbox, std::vector<Element*>& elements, int depth);
public:
    BoundingBox3f m_bbox;
    std::vector<Element*> m_elements;
    OctTreeNode* m_children[8];

    static int m_level;
    static int m_leaf;
    static int m_width;
    static int m_crossBBoxTriCount;
};

struct CompareOctTreeNodes {
    bool operator()(const std::pair<float, OctTreeNode*>& a, const std::pair<float, OctTreeNode*>& b) const {
        return a.first > b.first; // Min-heap: smallest distance has highest priority
    }
};

class OctTreeAccel : public Accel {
public:
    void build() override;
    bool rayIntersect(const Ray3f& ray, Intersection& its, bool shadowRay) const override;
public:
    OctTreeNode* m_root;
};

NORI_NAMESPACE_END
