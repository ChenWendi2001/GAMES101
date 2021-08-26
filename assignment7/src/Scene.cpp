//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // Contribution from the light source
    Intersection inter = intersect(ray);
    if(!inter.happened) return Vector3f(0,0,0);
    if(inter.m->hasEmission()) return inter.m->getEmission();
    Vector3f p = inter.coords;
    Vector3f wo = ray.direction;
    Vector3f N = inter.normal;

    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light,pdf_light);

    Vector3f x = inter_light.coords;
    Vector3f ws = (x-p).normalized();
    Vector3f NN = inter_light.normal;
    Vector3f emit = inter_light.emit;

    // Contribution from other reflectors
    Vector3f L_dir(0,0,0), L_indir(0,0,0);

    Ray p_to_x(p,ws);
    if((p-x).norm()-intersect(p_to_x).distance < 0.00016f)
        L_dir = emit * inter.m->eval(-wo,ws,N) * dotProduct(ws,N) * dotProduct(-ws, NN)/(p-x).norm()/(p-x).norm()/pdf_light;
    
    if(get_random_float()>RussianRoulette) return L_dir;

    Vector3f wi = inter.m -> sample(wo,N).normalized();
    Ray r(p,wi);
    Intersection object = intersect(r);
    if(object.happened && !object.m->hasEmission()){
        L_indir = castRay(r,depth+1) * inter.m->eval(-wo,wi,N) * dotProduct(wi, N) / inter.m->pdf(wo,wi,N) / RussianRoulette;
    }
    return L_dir + L_indir;

}