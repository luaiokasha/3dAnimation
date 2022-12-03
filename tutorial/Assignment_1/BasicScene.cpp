#include "BasicScene.h"
#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/vertex_triangle_adjacency.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
#include <vector>
#include "AutoMorphingModel.h"

using namespace std;
using namespace Eigen;
using namespace igl;
using namespace cg3d;




void BasicScene::level_up()
{
    (data->index)--;
    if (data->index < 0)
    {
        data->index = max(0, (data->current_available_collapses) - 1);
    }
    autoModel->meshIndex = (data->index);
}

void BasicScene::level_down()
{
    (data->index)++;
    if ((data->index) >= data->current_available_collapses)
    {
        (data->index) = 0;
    }
    autoModel->meshIndex = (data->index);
}


void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) }; // empty material
    // SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto bunnyMesh{ IglLoader::MeshFromFiles("bunny_igl","data/bunny.off") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };

    sphere1 = Model::Create("sphere", sphereMesh, material);
    bunny = Model::Create("bunny", bunnyMesh, material);
    cube = Model::Create("cube", cubeMesh, material);
    sphere1->Scale(2);
    sphere1->showWireframe = true;
    sphere1->Translate({ -3,0,0 });
    bunny->Translate({ 5,0,0 });
    bunny->Scale(0.12f);
    bunny->showWireframe = true;
    cube->showWireframe = true;
    camera->Translate(30, Axis::Z);
    auto mesh = sphere1->GetMeshList();
    data = new Data(mesh[0]->data[0].vertices, mesh[0]->data[0].faces);

    
    // Start of new code
    auto morph_function = [](Model* model, cg3d::Visitor* visitor)
    {
        int current_index = model->meshIndex;
        return (model->GetMeshList())[0]->data.size() * 0 + current_index;
    };
    autoModel = AutoMorphingModel::Create(*sphere1, morph_function);
    root->AddChild(autoModel);
    autoModel->Translate({ 0, 0, 25 });

    //basic_reset();
    modified_reset();
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);

    sphere1->Rotate(0.005f, Axis::Y);
    cube->Rotate(0.005f, Axis::Y);
    bunny->Rotate(0.005f, Axis::Y);
}


void BasicScene::set_mesh()
{
    igl::per_vertex_normals(data->V, data->F, data->VN);
    data->T = Eigen::MatrixXd::Zero(data->V.rows(), 2);
    auto mesh = autoModel->GetMeshList();
    mesh[0]->data.push_back({ data->V, data->F, data->VN, data->T });
    autoModel->SetMeshList(mesh);
    autoModel->meshIndex = data->index;
}

void BasicScene::basic_reset()
{
    if (data->manual_reset_selected)
    {
        data->manual_reset_selected = false;
        auto mesh = autoModel->GetMeshList();
        for (int i = 1; i < data->current_available_collapses; i++)
        {
            mesh[0]->data.pop_back();
        }
        autoModel->SetMeshList(mesh);
        data->current_available_collapses = 1;
    }
    data->F = data->OF;
    data->V = data->OV;
    igl::edge_flaps(data->F, data->E, data->EMAP, data->EF, data->EI);
    data->C.resize(data->E.rows(), data->V.cols());
    VectorXd costs(data->E.rows());
    // https://stackoverflow.com/questions/2852140/priority-queue-clear-method
    // original_Q.clear();
    data->original_Q = {};
    data->EQ = Eigen::VectorXi::Zero(data->E.rows());
    {
        Eigen::VectorXd costs(data->E.rows());
        igl::parallel_for(data->E.rows(), [&](const int e)
            {
                double cost = e;
                RowVectorXd p(1, 3);
                shortest_edge_and_midpoint(e, data->V, data->F, data->E, data->EMAP, data->EF, data->EI, cost, p);
                data->C.row(e) = p;
                costs(e) = cost;
            }, 10000);
        for (int e = 0; e < data->E.rows(); e++)
        {
            data->original_Q.emplace(costs(e), e, 0);
        }
    }
    data->num_collapsed = 0;
    data->index = 0;
    autoModel->meshIndex = data->index;
}

void BasicScene::basic_simplification()
{
    // If it isn't the last collapsed mesh, do nothing
    if (data->index != (data->current_available_collapses) - 1)
    {
        return;
    }
    // Collapse 10% of edges
    if (!data->original_Q.empty())
    {
        bool something_collapsed = false;
        // Collapse edge
        const int max_iter = std::ceil(0.1 * data->original_Q.size());
        for (int j = 0; j < max_iter; j++)
        {
            if (!collapse_edge(shortest_edge_and_midpoint, data->V, data->F, data->E, data->EMAP, data->EF, data->EI, data->original_Q, data->EQ, data->C))
            {
                break;
            }
            something_collapsed = true;
            data->num_collapsed++;
        }
        if (something_collapsed)
        {
            data->current_available_collapses++;
            data->index++;
            set_mesh();
        }
    }
}


void BasicScene::modified_reset()
{
    if (data->manual_reset_selected)
    {
        data->manual_reset_selected = false;
        auto mesh = autoModel->GetMeshList();
        for (int i = 1; i < data->current_available_collapses; i++)
        {
            mesh[0]->data.pop_back();
        }
        autoModel->SetMeshList(mesh);
        data->current_available_collapses = 1;
    }
    data->V = data->OV;
    data->F = data->OF;
    init();
    data->index = 0;
    autoModel->meshIndex = data->index;
}

void BasicScene::init()
{
    igl::edge_flaps(data->F, data->E, data->EMAP, data->EF, data->EI); // Init data_structures
    data->C.resize(data->E.rows(), data->V.cols());
    data->Q_iter.resize(data->E.rows()); // Number of edges 
    Q_matrix_calc();
    data->new_Q.clear();
    data->num_collapsed = 0;

    // Caculate egdes cost
    for (int i = 0; i < data->E.rows(); i++)
    {
        cost_calc(i);
    }
}

void BasicScene::Q_matrix_calc()
{
    std::vector<std::vector<int>> VF;  // Vertex vs Faces
    std::vector<std::vector<int>> VFi; // Non used ones
    int n = data->V.rows();
    data->Q_matrix.resize(n);
    igl::vertex_triangle_adjacency(n, data->F, VF, VFi);
    igl::per_face_normals(data->V, data->F, data->FN);

    for (int i = 0; i < n; i++)
    {
        // Initialize 
        data->Q_matrix[i] = Eigen::Matrix4d::Zero();

        // Caculate vertex Q matrix 
        for (int j = 0; j < VF[i].size(); j++)
        {
            // Get face normal
            Eigen::Vector3d normal = data->FN.row(VF[i][j]).normalized();

            // The equation is: ax+by+cz+d=0
            double a = normal[0];
            double b = normal[1];
            double c = normal[2];
            double d = data->V.row(i) * normal;
            d *= -1;

            // Kp = pp^T (s.t. p in planes)
            Eigen::Matrix4d Kp;
            Kp.row(0) = Eigen::Vector4d(a * a, a * b, a * c, a * d);
            Kp.row(1) = Eigen::Vector4d(a * b, b * b, b * c, b * d);
            Kp.row(2) = Eigen::Vector4d(a * c, b * c, c * c, c * d);
            Kp.row(3) = Eigen::Vector4d(a * d, b * d, c * d, d * d);
            data->Q_matrix[i] += Kp;
        }
    }
}

void BasicScene::cost_calc(int edge)
{
    // Vertexes of the edge
    int v1 = data->E(edge, 0);
    int v2 = data->E(edge, 1);
    Eigen::Matrix4d Q_edge = data->Q_matrix[v1] + data->Q_matrix[v2];

    // We will use this to find v' position
    Eigen::Matrix4d Q_position = Q_edge;
    Q_position.row(3) = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector4d v_position;
    double cost;
    bool isInversable;
    Q_position.computeInverseWithCheck(Q_position, isInversable);

    if (isInversable)
    {
        v_position = Q_position * (Eigen::Vector4d(0, 0, 0, 1));
        cost = v_position.transpose() * Q_edge * v_position;
    }
    else
    {
        // Find min error from v1, v2, (v1+v2)/2
        Eigen::Vector4d v1_position;
        v1_position << data->V.row(v1), 1;
        double cost1 = v1_position.transpose() * Q_edge * v1_position;

        Eigen::Vector4d v2_position;
        v2_position << data->V.row(v2), 1;
        double cost2 = v2_position.transpose() * Q_edge * v2_position;

        Eigen::Vector4d v1v2_position;
        v1v2_position << ((data->V.row(v1) + data->V.row(v2)) / 2), 1;
        double cost3 = v1v2_position.transpose() * Q_edge * v1v2_position;

        if (cost1 < cost2 && cost1 < cost3)
        {
            v_position = v1_position;
            cost = cost1;
        }
        else if (cost2 < cost1 && cost2 < cost3)
        {
            v_position = v2_position;
            cost = cost2;
        }
        else {
            v_position = v1v2_position;
            cost = cost3;
        }
    }
    Eigen::Vector3d new_position;
    new_position[0] = v_position[0];
    new_position[1] = v_position[1];
    new_position[2] = v_position[2];
    data->C.row(edge) = new_position;
    data->Q_iter[edge] = data->new_Q.insert(std::pair<double, int>(cost, edge)).first;
}

void BasicScene::modified_simplification()
{
    if (data->index != (data->current_available_collapses) - 1)
    {
        return;
    }
    bool something_collapsed = false;

    // Collapse 10% of the edges
    const int max_iter = std::ceil(0.1 * data->new_Q.size());
    for (int i = 0; i < max_iter; i++)
    {
        if (!modified_collapse_edge())
        {
            break;
        }
        something_collapsed = true;
        (data->num_collapsed)++;
    }
    if (something_collapsed)
    {
        (data->current_available_collapses)++;
        (data->index)++;
        set_mesh();
    }
}

bool BasicScene::modified_collapse_edge()
{
    PriorityQueue & curr_Q = data->new_Q;
    std::vector<PriorityQueue::iterator>& curr_Q_iter = data->Q_iter;
    int e1, e2, f1, f2; // Will be used in the igl collapse_edge function
    if (curr_Q.empty())
    {
        // No edges to collapse
        return false;
    }
    std::pair<double, int> pair = *(curr_Q.begin());
    if (pair.first == std::numeric_limits<double>::infinity())
    {
        // Min cost edge is infinite cost
        return false;
    }
    curr_Q.erase(curr_Q.begin()); // Delete from the queue
    int e = pair.second; // The lowest cost edge in the queue

    // The 2 vertices of the edge
    int v1 = data->E.row(e)[0];
    int v2 = data->E.row(e)[1];
    curr_Q_iter[e] = curr_Q.end();

    // Get the list of faces around the end point the edge
    std::vector<int> N = igl::circulation(e, true, data->EMAP, data->EF, data->EI);
    std::vector<int> Nd = igl::circulation(e, false, data->EMAP, data->EF, data->EI);
    N.insert(N.begin(), Nd.begin(), Nd.end());

    // Collapse the edage
    bool is_collapsed = igl::collapse_edge(e, data->C.row(e), data->V, data->F, data->E, data->EMAP, data->EF, data->EI, e1, e2, f1, f2);
    if (is_collapsed)
    {
        // Erase the two, other collapsed edges
        curr_Q.erase(curr_Q_iter[e1]);
        curr_Q_iter[e1] = curr_Q.end();
        curr_Q.erase(curr_Q_iter[e2]);
        curr_Q_iter[e2] = curr_Q.end();

        // Update the Q matrix for the 2 veterixes we collapsed 
        data->Q_matrix[v1] = data->Q_matrix[v1] + data->Q_matrix[v2];
        data->Q_matrix[v2] = data->Q_matrix[v1] + data->Q_matrix[v2];
        Eigen::VectorXd new_position;

        // Update local neighbors
        // Loop over original face neighbors
        for (auto n : N)
        {
            if (data->F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
                data->F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
                data->F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
            {
                for (int v = 0; v < 3; v++)
                {
                    // Get edge id
                    const int ei = data->EMAP(v * data->F.rows() + n);
                    // Erase old entry
                    curr_Q.erase(curr_Q_iter[ei]);
                    // Compute cost and potential placement and place in queue
                    cost_calc(ei);
                    new_position = data->C.row(ei);
                }
            }
        }
        cout << "Edge: " << e
            << ", Cost: " << pair.first
            << ", New Position: (" << new_position[0] << "," << new_position[1] << "," << new_position[2] << ")"
            << std::endl;
    }
    else
    {
        // Reinsert with infinite weight (the provided cost function must **not**
        // have given this un-collapsable edge inf cost already)
        pair.first = std::numeric_limits<double>::infinity();
        curr_Q_iter[e] = curr_Q.insert(pair).first;
    }
    return is_collapsed;
}
