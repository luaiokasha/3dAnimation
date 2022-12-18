#pragma once

#include "Scene.h"
#include <utility>
#include <iostream>
#include <vector>
#include <set>
#include <min_heap.h>
#include "Data.h"
#include <read_triangle_mesh.h>
#include <utility>
#include <igl/parallel_for.h>
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include <igl/per_face_normals.h>
#include <igl/vertex_triangle_adjacency.h>
#include <Eigen/Core>
#include "AutoMorphingModel.h"


using namespace std;
using namespace Eigen;
using namespace igl;
using namespace cg3d;
class BasicScene : public Scene
{
public:
    explicit BasicScene(string name, Display* display) : Scene(move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const Program& program, const Matrix4f& proj, const Matrix4f& view, const Matrix4f& model) override;
    void KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
   


    void set_mesh();
    void level_up();
    void level_down();
    void basic_reset();
    void basic_simplification();
    void modified_reset();
    void init_data();
    void Q_matrix_calc();
    void cost_calc(int edge);
    void modified_simplification();
    bool modified_collapse_edge();
    void place_objs(shared_ptr<cg3d::Model> obj1, shared_ptr<cg3d::Model> obj2);
private:
    shared_ptr<Movable> root;
    shared_ptr<Model> bunny, sphere1, cube;
    shared_ptr<Model> autoModel;
    Data * data;
    shared_ptr<Model> obj1;
    shared_ptr<Model> obj2;
    typedef set<pair<double, int>> PriorityQueue;
};

// Define a struct to represent the bounding box
struct BoundingBox
{
    Eigen::Vector3d min_corner;
    Eigen::Vector3d max_corner;
};