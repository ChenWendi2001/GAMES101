//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::min(std::max(float(0),u),float(1));
        v = std::min(std::max(float(0),v),float(1));

        auto u_img = u*width;
        auto v_img = (1-v)*height;

        float u_min = floor(u_img), u_max = ceil(u_img);
        float v_min = floor(v_img), v_max = ceil(v_img);

        auto c1 = (v_max-v_img)*image_data.at<cv::Vec3b>(v_min, u_min) + (v_img-v_min)*image_data.at<cv::Vec3b>(v_max, u_min) ;
        auto c2 = (v_max-v_img)*image_data.at<cv::Vec3b>(v_min, u_max) + (v_img-v_min)*image_data.at<cv::Vec3b>(v_max, u_max) ;

        auto c = (u_max-u_img)*c1+(u_img-u_min)*c2;

        return Eigen::Vector3f (c[0],c[1],c[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
