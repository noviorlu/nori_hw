#pragma once

#include <nori/bbox.h>
#include <nori/mesh.h>
#include <iostream>
#include <stack>
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

class TriangleIndex : public Element{
public:
    TriangleIndex(Mesh* mesh, int i) : Element(TRIANGLEIDX){
        m_mesh = mesh;
        idx = i;
        m_mesh->getTriangle(idx, m_vertices[0], m_vertices[1], m_vertices[2]);
    }
    
    bool inBBox(const BoundingBox3f& bbox)const override {
        for (int i = 0; i < 3; i++) {
            if (!bbox.contains(m_vertices[i])) {
                return false;
            }
        }
        return true;
    }

    bool rayIntersect(Ray3f& ray, Intersection& its) const override {
        float u, v, t;
        /* Find vectors for two edges sharing v[0] */
        Vector3f edge1 = m_vertices[1] - m_vertices[0], edge2 = m_vertices[2] - m_vertices[0];

        /* Begin calculating determinant - also used to calculate U parameter */
        Vector3f pvec = ray.d.cross(edge2);

        /* If determinant is near zero, ray lies in plane of triangle */
        float det = edge1.dot(pvec);

        if (det > -1e-8f && det < 1e-8f)
            return false;
        float inv_det = 1.0f / det;

        /* Calculate distance from v[0] to ray origin */
        Vector3f tvec = ray.o - m_vertices[0];

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
    Point3f m_vertices[3];
    Mesh* m_mesh;
    int idx;
};

#define MAX_DEPTH 10
#define MAX_WIDTH 16
#define RELASE_CONST 0.0
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
					std::cout << "; triangle index: " << ((TriangleIndex*)element)->idx;
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
					int idx = ((TriangleIndex*)element)->idx;
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

    int m_level;
};

struct CompareOctTreeNodes {
    bool operator()(const std::pair<float, OctTreeNode*>& a, const std::pair<float, OctTreeNode*>& b) const {
        return a.first > b.first; // Min-heap: smallest distance has highest priority
    }
};

NORI_NAMESPACE_END
