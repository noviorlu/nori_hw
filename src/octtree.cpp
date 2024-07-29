#include <nori/octtree.h>
#include <algorithm>
#include <queue>
NORI_NAMESPACE_BEGIN

OctTreeNode* OctTreeNode::build(const BoundingBox3f& bbox, std::vector<Element*>& elements, int depth)
{
    if(elements.size() == 0) return nullptr;
    OctTreeNode* node = new OctTreeNode(bbox);
    node->m_level = depth;
    if (depth >= MAX_DEPTH || elements.size() <= MAX_WIDTH)
    {
		node->m_elements = elements;
		return node;
	}

    // construct 8 children's bbox & elementList
    BoundingBox3f childrenBBox[8];
    std::vector<Element*> childrenElements[8];


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
                break;
            }
		}
        if (!poped) {
			node->m_elements.push_back(element);
		}
	}
    for (int i = 0; i < 8; i++) {
        if (childrenElements[i].size() == 0) node->m_children[i] = nullptr;
        else node->m_children[i] = build(childrenBBox[i], childrenElements[i], depth - 1);
    }
    
    depth++;
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

        bool isLeafHit = false;
        for (int i = 0; i < curNode->m_elements.size(); i++) {
            if (curNode->m_elements[i]->rayIntersect(ray, its)) {
                curHitElement = curNode->m_elements[i];
                isLeafHit = true;
            }
        }
        if (isLeafHit) { while (!pq.empty()) pq.pop(); };

        for (int i = 0; i < 8; i++) {
            auto child = curNode->m_children[i];
            if (child == nullptr || !child->m_bbox.rayIntersect(ray, nearT, farT)) continue;
			pq.push({ nearT, child });
		}
    } while (!pq.empty());
 //   if(!m_bbox.rayIntersect(ray)) return nullptr;
 //   for (int i = 0; i < m_elements.size(); i++) {
 //       if (m_elements[i]->rayIntersect(ray, its)) {
 //           curHitElement = m_elements[i];
	//	}
	//}

 //   std::map<float, OctTreeNode*> sortedChildren;
 //   for (int i = 0; i < 8; i++) {
 //       if (m_children[i] == nullptr) continue;
 //       float distance = m_children[i]->m_bbox.squaredDistanceTo(ray.o);
 //       sortedChildren.insert({ distance, m_children[i] });
 //   }

 //   for (auto& [distance, child] : sortedChildren) {
 //       auto cur = child->rayIntersect(ray, its);
 //       if (cur != nullptr) return cur;
 //   }

 //   for (int i = 0; i < 8; i++) {
 //       if(m_children[i] == nullptr) continue;
 //       auto hit = m_children[i]->rayIntersect(ray, its);
 //       if (hit != nullptr) curHitElement = hit;
	//}

    //std::stack<OctTreeNode*> m_stack;
    //if (!m_bbox.rayIntersect(ray)) return nullptr;
    //for (Element* element : m_elements) {
    //    bool hit = element->rayIntersect(ray, its);
    //    if (hit) curHitElement = element;
    //}

    //for (int i = 0; i < 8; i++) {
    //    if (m_children[i] != nullptr) {
    //        m_stack.push(m_children[i]);
    //    }
    //}
    //while (!m_stack.empty()) {
    //    OctTreeNode* node = m_stack.top();
    //    m_stack.pop();
    //    if (!node->m_bbox.rayIntersect(ray)) return nullptr;
    //    for (Element* element : node->m_elements) {
    //        bool hit = element->rayIntersect(ray, its);
    //        if(hit) curHitElement = element;
    //    }

    //    for (int i = 0; i < 8; i++) {
    //        if (node->m_children[i] != nullptr) {
    //            m_stack.push(node->m_children[i]);
    //        }
    //    }
    //}


	return curHitElement;
}

NORI_NAMESPACE_END
