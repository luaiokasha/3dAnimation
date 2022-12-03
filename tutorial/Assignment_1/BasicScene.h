#pragma once

#include "Scene.h"
#include <utility>
#include <min_heap.h>
#include "Data.h";
class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void set_mesh();
    void basic_reset();
    void basic_simplification();
    void level_up();
    void level_down();
    void modified_reset();
    void init();
    void Q_matrix_calc();
    void cost_calc(int edge);
    void modified_simplification();
    bool modified_collapse_edge();



private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> bunny, sphere1, cube;
    std::shared_ptr<cg3d::Model> autoModel;
    Data * data;
    typedef set<pair<double, int>> PriorityQueue;

    /*Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C;
    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;
    Eigen::MatrixXd VN, FN, T;
    igl::min_heap<std::tuple<double, int, int>> original_Q;
    int num_collapsed;
    int index;
    int current_available_collapses;
    bool manual_reset_selected;
    typedef std::set<std::pair<double, int>> PriorityQueue;
    PriorityQueue new_Q; // priority Queue: edge vs cost
    std::vector<PriorityQueue::iterator> Q_iter;
    std::vector <Eigen::Matrix4d> Q_matrix; //Vector: vertex vs Q matrix*/
};
