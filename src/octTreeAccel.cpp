#include <nori/octTreeAccel.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <queue>
#include <chrono>
NORI_NAMESPACE_BEGIN

int OctTreeNode::m_level = 0;
int OctTreeNode::m_leafCount = 0;
int OctTreeNode::m_nodeCount = 0;
int OctTreeNode::m_elementCount[MAX_WIDTH+1] = { 0 };
OctTreeNode* OctTreeNode::build(const BoundingBox3f& bbox, std::vector<Element*>& elements, int depth)
{
    if(elements.size() == 0) return nullptr;
    OctTreeNode* node = new OctTreeNode(bbox);
    OctTreeNode::m_level = OctTreeNode::m_level < depth ? depth : OctTreeNode::m_level;
    if (elements.size() <= MAX_WIDTH)
    {
        for (int i = 0; i < elements.size(); ++i) {
            node->m_elements[i] = elements[i];
        }
        OctTreeNode::m_leafCount++;
        OctTreeNode::m_elementCount[elements.size()]++;
		return node;
	}

    // construct 8 children's bbox & elementList
    BoundingBox3f childrenBBox[8];
    std::vector<Element*> childrenElements[8];
    std::vector<Element*> parentElements;

    Vector3f release = bbox.getExtents() * RELASE_CONST;
    Vector3f center = bbox.getCenter();
    for(int j = 0; j < 8; j++){
        Vector3f min, max;
        Vector3f corner = bbox.getCorner(j);

        for (int i = 0; i < 3; i++) {
            if (center[i] - corner[i] > 0) {
				min[i] = corner[i];
				max[i] = center[i] + release[i];
            }
            else {
                min[i] = center[i] - release[i];
                max[i] = corner[i];
            }
        }

        childrenBBox[j] = BoundingBox3f(min, max);
    }

    while (!elements.empty()) {
		Element* element = elements.back();
        elements.pop_back();

        bool poped = false;
        for (int i = 0; i < 8; i++) {
            if (element->inBBox(childrenBBox[i])) {
                childrenElements[i].push_back(element);
                poped = true;
            }
		}
        if (!poped) parentElements.push_back(element);
	}

    int crossBBox = (parentElements.size() > MAX_WIDTH) ? MAX_WIDTH : parentElements.size();
    for (int i = 0; i < crossBBox; i++) {
        node->m_elements[i] = parentElements[i];
    }

    for(int i = MAX_WIDTH; i < parentElements.size(); i++){
		for (int j = 0; j < 8; j++) {
			if(parentElements[i]->m_bbox.overlaps(childrenBBox[j])){
				childrenElements[j].push_back(parentElements[i]);
			}
		}
	}

    for (int i = 0; i < 8; i++) {
        if(childrenElements[i].size() == 0) continue;
        node->m_children[i] = build(childrenBBox[i], childrenElements[i], depth+1);
    }

    OctTreeNode::m_elementCount[crossBBox]++;
    return node;
}

Element* OctTreeNode::rayIntersect(Ray3f& ray, Intersection& its) const
{
    float nearT, farT;
    if (!m_bbox.rayIntersect(ray, nearT, farT)) return nullptr;

    Element* curHitElement = nullptr;
    OctTreeNode* curNode = nullptr;
    std::priority_queue<std::pair<float, OctTreeNode*>, std::vector<std::pair<float, OctTreeNode*>>, CompareOctTreeNodes> pq;
    pq.push({ nearT, const_cast<OctTreeNode*>(this) });
    do
    {
        curNode = pq.top().second;
        pq.pop();


        for (int i = 0; i < MAX_WIDTH; i++) {
            if (curNode->m_elements[i] == nullptr) break;
            if (curNode->m_elements[i]->rayIntersect(ray, its)) {
                curHitElement = curNode->m_elements[i];
            }
        }

        for (int i = 0; i < 8; i++) {
            auto child = curNode->m_children[i];
            if (child == nullptr || !child->m_bbox.rayIntersect(ray, nearT, farT)) continue;
            if(nearT < ray.maxt) pq.push({ nearT, child });
        }
    } while (!pq.empty());

	return curHitElement;
}


void OctTreeAccel::build()
{
    using namespace std::chrono;
    auto start = high_resolution_clock::now();

	std::vector<Element*> elements;
    int size = m_mesh->getTriangleCount();
    elements.reserve(size);

    for (int i = 0; i < size; i++) {
        elements.push_back(new Triangle(m_mesh, i));
    }
	m_root = OctTreeNode::build(m_bbox, elements, 0);

    //m_root->print(0);
    //OctTreeNode::validate(m_root, size);
    
    auto end = high_resolution_clock::now();
    std::cout << "OctTree build time:" << duration_cast<milliseconds>(end - start).count() << "ms\n";
    std::cout << "OctTree Depth: " << OctTreeNode::m_level << std::endl;
    std::cout << "LeafCount: " << OctTreeNode::m_leafCount << std::endl;
    std::cout << "NodeCount: " << OctTreeNode::m_nodeCount << std::endl;
    for(int i = 0; i <= MAX_WIDTH; i++){
		std::cout << "\t\tElementCount[" << i << "]: " << OctTreeNode::m_elementCount[i] << std::endl;
	}
    //exit(0);
}

bool OctTreeAccel::rayIntersect(const Ray3f& ray_, Intersection& its, bool shadowRay) const
{
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t idx = (uint32_t)-1;      // Triangle index of the closest intersection

    Ray3f ray(ray_);

    Element* element = m_root->rayIntersect(ray, its);
    if (element != nullptr) {
        foundIntersection = true;
    }

    if (foundIntersection) {
        Triangle* triangle = (Triangle*)element;
        idx = triangle->idx;
        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh* mesh = its.mesh;
        const MatrixXf& V = mesh->getVertexPositions();
        const MatrixXf& N = mesh->getVertexNormals();
        const MatrixXf& UV = mesh->getVertexTexCoords();
        const MatrixXu& F = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t i0 = F(0, idx), i1 = F(1, idx), i2 = F(2, idx);
        Point3f p0 = V.col(i0), p1 = V.col(i1), p2 = V.col(i2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(i0) +
            bary.y() * UV.col(i1) +
            bary.z() * UV.col(i2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(i0) +
                    bary.y() * N.col(i1) +
                    bary.z() * N.col(i2)).normalized());
        }
        else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}


NORI_NAMESPACE_END

