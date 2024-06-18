//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

int g_totals = 0;
std::mutex g_mutex;

void render_thread(const Scene& scene, std::vector<Vector3f>& framebuffer, int startRow, 
                   int endRow, float imageAspectRatio, float scale, int spp, Vector3f eye_pos) 
{
    int m = startRow * scene.width;
    for (uint32_t j = startRow; j < endRow; ++ j) 
    {
        for (uint32_t i = 0; i < scene.width; ++ i) 
        {
            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));
            for (int k = 0; k < spp; k++)
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            m ++;
        }
        g_mutex.lock();
        g_totals ++;
        UpdateProgress(g_totals / (float)scene.height);
        g_mutex.unlock();
    }
}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 512; // default 16
    std::cout << "SPP: " << spp << "\n";

    int numThreads = std::thread::hardware_concurrency();
    std::cout << "Runnning on " << numThreads << " CPUs" << std::endl;

	int heightPerThread = std::ceil(scene.height / numThreads);
    std::vector<std::thread> workers;

    for (int i = 0; i < numThreads; i++)
	{
        int st = i * heightPerThread;
        int ed = (i + 1) * heightPerThread;
        if (i == numThreads - 1) {
            ed = scene.height;
        }
        std::cout << "id:" << i << " " << st << "=>" << ed << std::endl;
        workers.push_back(std::thread(render_thread, std::ref(scene), std::ref(framebuffer), 
                                    st, ed, imageAspectRatio, scale, spp, eye_pos));
	}

    for (int i =0;i < workers.size();i++)
    {
        workers[i].join();
    }

    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
