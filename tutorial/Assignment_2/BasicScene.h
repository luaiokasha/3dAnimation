#pragma once

#include "Scene.h"
#include "igl/AABB.h"

#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void AlignedBoxTransformation(Eigen::AlignedBox<double, 3>& aligned_box, std::shared_ptr<cg3d::Model> cube_model);
    bool Collision(igl::AABB<Eigen::MatrixXd, 3>* aligned_box1, igl::AABB<Eigen::MatrixXd, 3>* aligned_box2, int level);
    bool Boxes_Intersection(Eigen::AlignedBox<double, 3>& aligned_box1, Eigen::AlignedBox<double, 3>& aligned_box2);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cyl, sphere1, cube;
    std::shared_ptr<cg3d::Model> object1, object2, cube1, cube2, hit_cube1, hit_cube2;
    std::shared_ptr<cg3d::Model> autoModel1, autoModel2;
    std::vector<std::shared_ptr<cg3d::Model>> autoModels;    
    bool got_collision;
    Eigen::MatrixXi F1, F2;
    Eigen::MatrixXd V1, V2;
    igl::AABB<Eigen::MatrixXd, 3> Tree1, Tree2;
    float velocity_x, velocity_y, obj1_rotation_z, obj2_rotation_z;
};
