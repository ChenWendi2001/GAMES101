#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch(splitMethod){
            case SplitMethod::NAIVE:{
                switch (dim) {
                case 0:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x <
                            f2->getBounds().Centroid().x;
                    });
                    break;
                case 1:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y <
                            f2->getBounds().Centroid().y;
                    });
                    break;
                case 2:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z <
                            f2->getBounds().Centroid().z;
                    });
                    break;
                }
                int idx = (objects.size() / 2);
                auto beginning = objects.begin();
                auto middling = objects.begin() + idx;
                auto ending = objects.end();

                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);

                assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                node->left = recursiveBuild(leftshapes);
                node->right = recursiveBuild(rightshapes);

                node->bounds = Union(node->left->bounds, node->right->bounds);
                break;
            }
            case SplitMethod::SAH:{
                float C_min  = 0xffff;
                float S_N = centroidBounds.SurfaceArea();

                const float C_trav = 1.0;
                const float C_isect = 2.0;
                const int bucket_num = 16;

                int bucket_split = 0;

                Bounds3 bucket[bucket_num];
                int count[bucket_num];
                for(int i = 0;i<bucket_num;++i){
                    bucket[i] = Bounds3();
                    count[i] = 0;
                }

                for(int i = 0;i<objects.size();++i){
                    int bucket_idx = 0;
                    if(dim == 0)
                        bucket_idx = (bucket_num-1) * centroidBounds.Offset(objects[i]->getBounds().Centroid()).x;
                    else if(dim == 1)
                        bucket_idx = (bucket_num-1)  * centroidBounds.Offset(objects[i]->getBounds().Centroid()).y;
                    else if(dim == 2)
                        bucket_idx = (bucket_num-1)  * centroidBounds.Offset(objects[i]->getBounds().Centroid()).z;
                    
                    bucket[bucket_idx] = Union(bucket[bucket_idx],objects[i]->getBounds().Centroid());
                    count[bucket_idx]++;
                }

                for(int i = 1; i<bucket_num;++i){
                    Bounds3 A,B;
                    int N_A = 0, N_B = 0;
                    for(int j = 0;j<i;++j){
                            A = Union(A,bucket[j]);
                            N_A += count[j];
                    }
                    for(int j = i;j<bucket_num;++j){   
                            B = Union(B,bucket[j]);
                            N_B += count[j];
                    }
                    float C_cur = C_trav + C_isect*(A.SurfaceArea()*N_A+B.SurfaceArea()*N_B)/S_N;
                    if(C_cur<C_min){
                        C_min = C_cur;
                        bucket_split = i;
                        // std::cout<<"SAH\n";
                    }
                }
                std::vector<Object*> leftshapes,rightshapes;
                for(int i = 0 ; i < objects.size();++i)
                {
                    int bucket_idx = 0;
                    if(dim == 0)
                        bucket_idx = (bucket_num-1) * centroidBounds.Offset(objects[i]->getBounds().Centroid()).x;
                    else if(dim == 1)
                        bucket_idx = (bucket_num-1)  * centroidBounds.Offset(objects[i]->getBounds().Centroid()).y;
                    else if(dim == 2)
                        bucket_idx = (bucket_num-1)  * centroidBounds.Offset(objects[i]->getBounds().Centroid()).z;
                    if(bucket_idx < bucket_split)
                        leftshapes.push_back(objects[i]);
                    else
                        rightshapes.push_back(objects[i]);
                }
                assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                node->left = recursiveBuild(leftshapes);
                node->right = recursiveBuild(rightshapes);

                node->bounds = Union(node->left->bounds, node->right->bounds);
                break;
            }
        }
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Vector3f invDir(1/ray.direction.x,1/ray.direction.y,1/ray.direction.z);
    std::array<int,3> dirIsNeg = {int(ray.direction.x>0),int(ray.direction.y>0),int(ray.direction.z>0)};

    if(!node->bounds.IntersectP(ray,invDir,dirIsNeg)) return {};

    if(node->left==nullptr && node->right==nullptr)
        return node->object->getIntersection(ray);
    
    Intersection hit1 = getIntersection(node->left,ray);
    Intersection hit2 = getIntersection(node->right,ray);

    return hit1.distance<hit2.distance ? hit1:hit2;

}