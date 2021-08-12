#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    rotation_angle = rotation_angle / 180.0 * MY_PI;

    model(0,0) = cos(rotation_angle);
    model(0,1) = -sin(rotation_angle);
    model(1,0) = sin(rotation_angle);
    model(1,1) = cos(rotation_angle);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f p_to_o ;
    p_to_o << zNear, 0, 0, 0,
                0, zNear, 0, 0,
                0, 0, zNear+zFar, -zNear*zFar,
                0, 0, 1, 0;

    float r, l, t, b;
    eye_fov = eye_fov/180.0 * MY_PI;
    t = tan(eye_fov/2.0) * zNear;
    b = -t;
    r = t * aspect_ratio;
    l = -r;

    Eigen::Matrix4f o_1 = Eigen::Matrix4f::Identity(), o_2 = Eigen::Matrix4f::Identity();

    o_1(0,3) = -(r+l)/2.0;
    o_1(1,3) = -(t+b)/2.0;
    o_1(2,3) = -(zNear+zFar)/2.0;

    o_2(0,0) = 2.0/(r-l);
    o_2(1,1) = 2.0/(t-b);
    o_2(2,2) = 2.0/(zNear-zFar);

    projection = o_2 * o_1 * p_to_o;

    return projection;
}

Eigen::Matrix4f get_rotation (Vector3f axis, float angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity(), R, N;
    angle = angle /180.0 * MY_PI;

    N << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;
    R = cos(angle)*I + (1-cos(angle))*axis*axis.adjoint() + sin(angle)*N;
    for(int i = 0;i<3;i++){
        for(int j = 0;j<3;j++){
            model(i,j) = R(i,j);
        }
    }

    return model;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation (Vector3f(0,0,1), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
