/*
 * @Description: Description of this file
 * @Version: 2.0
 * @Author: xm
 * @Date: 2020-03-03 11:13:18
 * @LastEditors: xm
 * @LastEditTime: 2024-06-14 23:29:04
 */
// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#define SSAA true

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f A(_v[0].x(), _v[0].y());
    Vector2f B(_v[1].x(), _v[1].y());
    Vector2f C(_v[2].x(), _v[2].y());

    // Calculate the vector
    Vector2f AB =  B - A;
    Vector2f BC =  C - B;
    Vector2f CA =  A - C;

    Vector2f P(x, y);
    Vector2f AP = P - A;
    Vector2f BP = P - B;
    Vector2f CP = P - C;

    // check by cross-product
    float z1 = AB.x() * AP.y() - AB.y() * AP.x();
    float z2 = BC.x() * BP.y() - BC.y() * BP.x();
    float z3 = CA.x() * CP.y() - CA.y() * CP.x();

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            // vec /= vec.w();
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }

    //SSAA Begin
    if (SSAA) 
    {
        for (int x = 0; x < width; x++) 
        {
            for (int y = 0; y < height; y++)
            {
                Eigen::Vector3f color(0, 0, 0);
                for (int i = 0; i < 4; i++)
                    color += frame_buf_2xSSAA[get_index(x, y)][i];
                color /= 4;
                set_pixel(Eigen::Vector3f(x,y,1.0f), color);
            }           
        }
    }
    //SSAA End
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // var v includes coordinates of the triangle's vertices
    // v[i][j], i denotes i-th vertex, j = 0, 1, 2 denotes x, y, z, respectively
    float min_x = std::floor(std::min(v[0][0], std::min(v[1][0], v[2][0])));
    float max_x = std::ceil(std::max(v[0][0], std::max(v[1][0], v[2][0])));
    float min_y = std::floor(std::min(v[0][1], std::min(v[1][1], v[2][1])));
    float max_y = std::ceil(std::max(v[0][1], std::max(v[1][1], v[2][1])));
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = min_x; x < max_x; x ++)
        for (int y = min_y; y < max_y; y ++)
            if (SSAA)
            {
                int child_idx = 0;
                for (float i = 0.25; i < 1.0; i += 0.5)
                    for (float j = 0.25; j < 1.0; j += 0.5)
                    {
                        if (insideTriangle(x + i, y + j, t.v))
                        {
                            auto [alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            int buf_index = get_index(x,y);
                            if(z_interpolated >= depth_buf_2xSSAA[buf_index][child_idx]) continue;
                            depth_buf_2xSSAA[buf_index][child_idx] = z_interpolated;
                            frame_buf_2xSSAA[buf_index][child_idx] = t.getColor();
                        }
                        child_idx ++;
                    }
            }
            else
            {
                if (insideTriangle(x + 0.5, y + 0.5, t.v))
                {
                    // If so, use the following code to get the interpolated z value.
                    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    //z_interpolated *= w_reciprocal;
                    auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    int buf_index = get_index(x,y);
                    if(z_interpolated >= depth_buf[buf_index]) continue;
                    depth_buf[buf_index] = z_interpolated;
                    set_pixel(Vector3f(x, y, 1.0f), t.getColor());
                }
            }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        //SSAA Begin
        for (int i = 0; i < frame_buf_2xSSAA.size(); i++) {
            frame_buf_2xSSAA[i].resize(4);
            std::fill(frame_buf_2xSSAA[i].begin(), frame_buf_2xSSAA[i].end(), Eigen::Vector3f{ 0, 0, 0 });
        }
        //SSAA End
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //SSAA Begin
        for (int i = 0; i < depth_buf_2xSSAA.size(); i++) {
            depth_buf_2xSSAA[i].resize(4);
            std::fill(depth_buf_2xSSAA[i].begin(), depth_buf_2xSSAA[i].end(), std::numeric_limits<float>::infinity());
        }
        //SSAA End
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //SSAA Begin
    frame_buf_2xSSAA.resize(w * h);
    depth_buf_2xSSAA.resize(w * h);
    //SSAA End
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on