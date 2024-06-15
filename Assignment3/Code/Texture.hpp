/*
 * @Description: Description of this file
 * @Version: 2.0
 * @Author: xm
 * @Date: 2020-03-04 12:51:24
 * @LastEditors: xm
 * @LastEditTime: 2024-06-15 15:35:09
 */
//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        // Limit the range of u and v coordinate values ​​to the [0,1] interval
        if (u < 0)
            u = 0;
        if (u > 1)
            u = 1;
        if (v < 0)
            v = 0;
        if (v > 1)
            v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        // Limit the range of u and v coordinate values ​​to the [0,1] interval
        if (u < 0)
            u = 0;
        if (u > 1)
            u = 1;
        if (v < 0)
            v = 0;
        if (v > 1)
            v = 1;

        // transfer texture coords to image coords
        float u_img = u * width;
        float v_img = (1 - v) * height;

        // Calculate the nearest four pixel coordinates
        int u0 = std::floor(u_img);
        int u1 = std::min(u0 + 1, width - 1);
        int v0 = std::floor(v_img);
        int v1 = std::min(v0 + 1, height - 1);

        // Calculate s and t, which are the relative positions from u0 to u1 and v0 to v1
        float s = u_img - u0;
        float t = v_img - v0;

        // Get the colors of the surrounding four pixels
        auto color00 = image_data.at<cv::Vec3b>(v0, u0);
        auto color01 = image_data.at<cv::Vec3b>(v0, u1);
        auto color10 = image_data.at<cv::Vec3b>(v1, u0);
        auto color11 = image_data.at<cv::Vec3b>(v1, u1);

        // Perform linear interpolation in the u direction
        auto color0 = color00 * (1 - s) + color01 * s;
        auto color1 = color10 * (1 - s) + color11 * s;

        // Perform linear interpolation in the v direction
        auto color = color0 * (1 - t) + color1 * t;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
