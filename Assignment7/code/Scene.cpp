//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

// Check whether the light intersects with a surround box in the current scene
Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

// Randomly sample light sources in the scene
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    // Luminous Area
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // Sample a point uniformly by area on all light sources and calculate the probability
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            // Sample the light source and obtain the position and pdf (uniform sampling 1/S)
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
    // Backward Ray Tracing
    // 1. First, we shoot spp rays from each pixel (eventually averaging the results)
    // 2. Calculate whether the light hits object. If not, return background color directly.
    // 3.1 Denote intersection point as p. 
    // 3.2 Sample a light source x from illuminated objects.
    // 4. If no obstruction between p and x, calculate the direct lighting dir_L from x to p.
    // 5.1 Calculate indirect illumination according to Russian roulette. 
    // 5.2 Sample a direction r in the hemisphere over p for diffuse reflection.
    // 6. If r hits a non-light source object, calling castRay recursively to obtain indirect lighting indir_L.
    // 7. Return the light of p + dir_L + indir_L

    // TO DO Implement Path Tracing Algorithm here
    // init the color as background color
    Vector3f hitColor = this->backgroundColor;

    // Check whether the light intersects any object in the scene
    Intersection inter = Scene::intersect(ray);
    // If no intersection, return the background color
    if (!inter.happened)
        return hitColor;

    // If ray intersect with objects
    // Get the coords, in_dir, and normal of intersection
    Vector3f inter_coords = inter.coords;
    Vector3f inter_ray_dir = ray.direction;
    Vector3f inter_normal = inter.normal;
    Vector3f L_dir(0), L_indir(0);

    // avoid self-intersection
    Vector3f p_deviation = (dotProduct(ray.direction, inter_normal) < 0) ?
            inter_coords + inter_normal * EPSILON : inter_coords - inter_normal * EPSILON;

    switch (inter.m->getType())
        {
            case MIRROR:
            {
                // Calculate indirect lighting, Russian Roulette
                float ksi = get_random_float();
                if (ksi < RussianRoulette)
                {
                    // sample the reflection direction
                    Vector3f wi = normalize(inter.m->sample(inter_ray_dir, inter_normal));
                    // Shoot a ray
                    Ray r(p_deviation, wi);
                    //If ray r hit a object at q
                    Intersection bounce_point_inter = Scene::intersect(r);
                    if (bounce_point_inter.happened)
                    {
                        float pdf = inter.m->pdf(inter_ray_dir, wi, inter_normal);
                        if (pdf > EPSILON)
                            L_indir = castRay(r, depth + 1) * inter.m->eval(inter_ray_dir, wi, inter_normal) * dotProduct(wi, inter_normal) / (pdf * RussianRoulette);
                    }
                }
                break;
            }
            default:
            {
                // Randomly select a sampling point from the light source in the scene
                Intersection light_point_inter;
                float pdf_light;
                sampleLight(light_point_inter, pdf_light);

                // Get the coords, normal and light intensity of sampling point x 
                Vector3f sampled_coords = light_point_inter.coords;
                Vector3f sampled_normal = light_point_inter.normal;
                Vector3f sampled_emit = light_point_inter.emit;

                // Calculate the direction vector from inter to sampling point
                Vector3f p2x_dir = normalize(sampled_coords - inter_coords);
                float p2x_dist = (sampled_coords - inter_coords).norm();

                // Shoot a ray from inter to Sampling point 
                Ray ray_pTox(p_deviation, p2x_dir);

                // If the ray is not blocked, calculate direct lighting
                Intersection blocked_point_inter = Scene::intersect(ray_pTox);
                if (abs(p2x_dist - blocked_point_inter.distance < 0.01))
                {
                    Vector3f fr = inter.m->eval(inter_ray_dir, p2x_dir, inter_normal);
                    Vector3f obj_cos = dotProduct(p2x_dir, inter_normal);
                    Vector3f light_cos = dotProduct(-p2x_dir, sampled_normal);
                    L_dir = sampled_emit * fr * obj_cos * light_cos / (p2x_dist * p2x_dist * pdf_light);
                }

                // Calculate indirect lighting, Russian Roulette
                float ksi = get_random_float();
                if (ksi < RussianRoulette)
                {
                    // Randomly sample a diffuse direction
                    Vector3f wi = normalize(inter.m->sample(inter_ray_dir, inter_normal));
                    // shoot a ray from p (intersection point)
                    Ray r(p_deviation, wi);
                    // If ray r hit a non-emitting object
                    Intersection bounce_point_inter = Scene::intersect(r);
                    if (bounce_point_inter.happened && !bounce_point_inter.m->hasEmission())
                    {
                        float pdf = inter.m->pdf(inter_ray_dir, wi, inter_normal);
                        Vector3f fr = inter.m->eval(inter_ray_dir, wi, inter_normal);
                        if(pdf > EPSILON)
                            L_indir = castRay(r, depth + 1) * fr * dotProduct(wi, inter_normal) / (pdf *RussianRoulette);
                    }
                }
                break;
            }
        }

    hitColor = inter.m->getEmission() + L_dir + L_indir;
    // Reduce white noise
    hitColor.x = clamp(0, 1, hitColor.x);
    hitColor.y = clamp(0, 1, hitColor.y);
    hitColor.z = clamp(0, 1, hitColor.z);
    return hitColor;
}