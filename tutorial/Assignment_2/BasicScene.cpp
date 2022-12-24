#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"
#include "igl/per_vertex_normals.h"
#include "igl/vertex_triangle_adjacency.h"
#include "igl/per_face_normals.h"
#include "igl/circulation.h"
#include "igl/collapse_edge.h"

#include "AutoMorphingModel.h"

using namespace cg3d;
using namespace std;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); 
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };
    auto empty_material{ std::make_shared<Material>("material", program) }; // empty material
    auto hit_material{ std::make_shared<Material>("material", program) }; // hit material

    material->AddTexture(0, "textures/box0.bmp", 2);
    hit_material->AddTexture(0, "textures/box0.bmp", 2); // make it more visible

    // Sphere Meshes
    auto sphereMesh1{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto sphereMesh2{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };

    // Bunny Meshes
    auto bunnylMesh1{ IglLoader::MeshFromFiles("cyl_igl","data/bunny.off") };
    auto bunnylMesh2{ IglLoader::MeshFromFiles("cyl_igl","data/bunny.off") };

    // Cube Meshes
    auto cubeMesh1{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto cubeMesh2{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto cubeMesh3{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto cubeMesh4{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };

    
    camera->Translate(10, Axis::Z);

    

    // Bunnies
    object1 = Model::Create("bunny1", bunnylMesh1, material);
    object2 = Model::Create("bunny2", bunnylMesh2, material);
    object1->showWireframe = true;
    object2->showWireframe = true;

    // Biggest bounding boxes
    cube1 = Model::Create("cube1", cubeMesh1, empty_material);
    cube2 = Model::Create("cube2", cubeMesh2, empty_material);

    // Smallest bounding boxes
    hit_cube1 = Model::Create("cube3", cubeMesh3, hit_material);
    hit_cube2 = Model::Create("cube4", cubeMesh4, hit_material);

    auto morph_function = [](Model* model, cg3d::Visitor* visitor)
    {
        int current_index = model->meshIndex;
        return (model->GetMeshList())[0]->data.size() * 0 + current_index;
    };
    autoModel1 = AutoMorphingModel::Create(*object1, morph_function);
    autoModel2 = AutoMorphingModel::Create(*object2, morph_function);

    root->AddChild(autoModel1);
    root->AddChild(autoModel2);

    autoModel1->AddChild(cube1);
    autoModel2->AddChild(cube2);
    cube1->showFaces = false;
    cube2->showFaces = false;
    cube1->showWireframe = true;
    cube2->showWireframe = true;

    // Spheres
    //autoModel1->Translate({ -1.5, 0, 0 });
    //autoModel2->Translate({ 1.5, 0, 0 });

    // Bunnies
    autoModel1->Translate({ -0.3, -0.1, 9 });
    autoModel2->Translate({ 0.3, -0.1, 9 });

    // Initialize object1 tree
    auto mesh = autoModel1->GetMeshList();
    V1 = mesh[0]->data[0].vertices;
    F1 = mesh[0]->data[0].faces;
    Tree1.init(V1, F1);

    // Initialize object2 tree
    mesh = autoModel2->GetMeshList();
    V2 = mesh[0]->data[0].vertices;
    F2 = mesh[0]->data[0].faces;
    Tree2.init(V2, F2);

    // Creating each object's biggest bounding box 
    AlignedBoxTransformation(Tree1.m_box, cube1);
    AlignedBoxTransformation(Tree2.m_box, cube2);

    got_collision = true;
    velocity_x = 0.001;
    velocity_y = 0.0;
    obj1_rotation_z = 0.001;
    obj2_rotation_z = 0.0;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);

    autoModel1->Translate({ velocity_x, velocity_y, 0 });

    if (got_collision && Collision(&Tree1, &Tree2, 0))
    {
        autoModel1->Rotate(0.0, Axis::Z);
        autoModel2->Rotate(0.0, Axis::Z);

        velocity_x = 0.0;
        velocity_y = 0.0;

        obj1_rotation_z = 0.0;
        obj2_rotation_z = 0.0;

        got_collision = false;
        std::cout << "Collision, tracackckkckckcc!" << std::endl;
    }
    
    else
    {
        autoModel1->Rotate(obj1_rotation_z, Axis::Z);
        autoModel2->Rotate(obj2_rotation_z, Axis::Z);


    }

}

void BasicScene::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key)
        {
        case GLFW_KEY_SPACE:
            velocity_x = 0.0;
            velocity_y = 0.0;
            break;
        case GLFW_KEY_UP:
            got_collision = true;
            velocity_x = 0.0;
            velocity_y = 0.001;
            obj1_rotation_z = 0.0;
            obj2_rotation_z = 0.001;
            break;
        case GLFW_KEY_DOWN:
            got_collision = true;
            velocity_x = 0.0;
            velocity_y = -0.001;
            obj1_rotation_z = 0.0;
            obj2_rotation_z = 0.001;
            break;
        case GLFW_KEY_RIGHT:
            got_collision = true;
            velocity_x = 0.001;
            velocity_y = 0.0;
            obj1_rotation_z = -0.001;
            obj2_rotation_z = 0.001;
            break;
        case GLFW_KEY_LEFT:
            got_collision = true;
            velocity_x = -0.001;
            velocity_y = 0.0;
            obj1_rotation_z = 0.001;
            obj2_rotation_z = 0.001;
            break;
        }
    }
}

void BasicScene::AlignedBoxTransformation(Eigen::AlignedBox<double, 3>& aligned_box, std::shared_ptr<cg3d::Model> cube_model)
{
    // Get the aligned box coordinates
    Eigen::RowVector3d BottomRightCeil = aligned_box.corner(aligned_box.BottomRightCeil);
    Eigen::RowVector3d BottomRightFloor = aligned_box.corner(aligned_box.BottomRightFloor);
    Eigen::RowVector3d BottomLeftCeil = aligned_box.corner(aligned_box.BottomLeftCeil);
    Eigen::RowVector3d BottomLeftFloor = aligned_box.corner(aligned_box.BottomLeftFloor);
    Eigen::RowVector3d TopRightCeil = aligned_box.corner(aligned_box.TopRightCeil);
    Eigen::RowVector3d TopRightFloor = aligned_box.corner(aligned_box.TopRightFloor);
    Eigen::RowVector3d TopLeftCeil = aligned_box.corner(aligned_box.TopLeftCeil);
    Eigen::RowVector3d TopLeftFloor = aligned_box.corner(aligned_box.TopLeftFloor);



    // Update the cube with the aligned box coordinates
    Eigen::MatrixXd V, VN, T;
    Eigen::MatrixXi F;

    V.resize(8, 3);
    F.resize(12, 3);
    V.row(0) = TopLeftCeil;
    V.row(1) = TopRightCeil;
    V.row(2) = BottomLeftCeil;
    V.row(3) = BottomRightCeil;
    V.row(4) = TopLeftFloor;
    V.row(5) = TopRightFloor;
    V.row(6) = BottomLeftFloor;
    V.row(7) = BottomRightFloor;

    F.row(0) = Eigen::Vector3i(6, 7, 5);
    F.row(1) = Eigen::Vector3i(5, 4, 6);
    F.row(2) = Eigen::Vector3i(2, 3, 1);
    F.row(3) = Eigen::Vector3i(1, 0, 2);
    F.row(4) = Eigen::Vector3i(6, 4, 0);
    F.row(5) = Eigen::Vector3i(0, 4, 6);
    F.row(6) = Eigen::Vector3i(3, 1, 5);
    F.row(7) = Eigen::Vector3i(3, 5, 7);
    F.row(8) = Eigen::Vector3i(4, 0, 1);
    F.row(9) = Eigen::Vector3i(1, 5, 4);
    F.row(10) = Eigen::Vector3i(2, 3, 7);
    F.row(11) = Eigen::Vector3i(7, 6, 2);
    igl::per_vertex_normals(V, F, VN);
    T = Eigen::MatrixXd::Zero(V.rows(), 2);

    auto mesh = cube_model->GetMeshList();
    mesh[0]->data.push_back({ V, F, VN, T });
    cube_model->SetMeshList(mesh);
    cube_model->meshIndex += 1;
}

bool BasicScene::Collision(igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2, int level)
{
    if (tree1 == nullptr || tree2 == nullptr) 
    {
        return false;
    }
    if (!Boxes_Intersection(tree1->m_box, tree2->m_box))
    {
        return false;
    }

    // in case of boxes intersection, minimize the bb for both objs
    if (tree1->is_leaf() && tree2->is_leaf()) {
        AlignedBoxTransformation(tree1->m_box, hit_cube1);
        AlignedBoxTransformation(tree2->m_box, hit_cube2);
        autoModel1->AddChild(hit_cube1);
        autoModel2->AddChild(hit_cube2);
        hit_cube1->showFaces = false;
        hit_cube2->showFaces = false;
        hit_cube1->showWireframe = true;
        hit_cube2->showWireframe = true;
        return true;
    }
    if (tree1->is_leaf() && !tree2->is_leaf()) {
        return 
            Collision(tree1, tree2->m_right, level + 1) ||
            Collision(tree1, tree2->m_left, level + 1);
    }
    if (!tree1->is_leaf() && tree2->is_leaf()) {
        return 
            Collision(tree1->m_right, tree2, level + 1) ||
            Collision(tree1->m_left, tree2, level + 1);
    }
    return 
        Collision(tree1->m_left, tree2->m_left, level + 1) ||
        Collision(tree1->m_left, tree2->m_right, level + 1) ||
        Collision(tree1->m_right, tree2->m_left, level + 1) ||
        Collision(tree1->m_right, tree2->m_right, level + 1);
}

bool BasicScene::Boxes_Intersection(Eigen::AlignedBox<double, 3>& aligned_box1, Eigen::AlignedBox<double, 3>& aligned_box2)
{
    // Matrix A
    Eigen::Matrix3d A = autoModel1->GetRotation().cast<double>();
    Eigen::Vector3d A0 = A.col(0);
    Eigen::Vector3d A1 = A.col(1);
    Eigen::Vector3d A2 = A.col(2);

    // Matrix B
    Eigen::Matrix3d B = autoModel2->GetRotation().cast<double>();
    Eigen::Vector3d B0 = B.col(0);
    Eigen::Vector3d B1 = B.col(1);
    Eigen::Vector3d B2 = B.col(2);

    // Matrix C 
    Eigen::Matrix3d C = A.transpose() * B; //C = A ^ T * B
    
    // Get the lengths of the sides of bb
    Eigen::Vector3d a = aligned_box1.sizes();
    Eigen::Vector3d b = aligned_box2.sizes();
    
    // lengths' halfs
    a = a / 2;
    b = b / 2;

    // Matrix D
    Eigen::Vector4d CenterA = Eigen::Vector4d(aligned_box1.center()[0], aligned_box1.center()[1], aligned_box1.center()[2], 1);
    Eigen::Vector4d CenterB = Eigen::Vector4d(aligned_box2.center()[0], aligned_box2.center()[1], aligned_box2.center()[2], 1);
    Eigen::Vector4d D4d = autoModel2->GetTransform().cast<double>() * CenterB - autoModel1->GetTransform().cast<double>() * CenterA;
    Eigen::Vector3d D = D4d.head(3);

    float R0, R1, R;

    // Check the 15 conditions due to the table

    
    //A0
    R0 = a(0);
    R1 = b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(0)(1)) + b(2) * abs(C.row(0)(2));
    R = abs(A0.transpose() * D);
    if (R0 + R1 < R) 
        return false;
    
    //A1
    R0 = a(1);
    R1 = b(0) * abs(C.row(1)(0)) + b(1) * abs(C.row(1)(1)) + b(2) * abs(C.row(1)(2));
    R = abs(A1.transpose() * D);
    if (R0 + R1 < R) 
        return false;

    //A2
    R0 = a(2);
    R1 = b(0) * abs(C.row(2)(0)) + b(1) * abs(C.row(2)(1)) + b(2) * abs(C.row(2)(2));
    R = abs(A2.transpose() * D);
    if (R0 + R1 < R) 
        return false;

    //B0
    R0 = a(0) * abs(C.row(0)(0)) + a(1) * abs(C.row(1)(0)) + a(2) * abs(C.row(2)(0));
    R1 = b(0);
    R = abs(B0.transpose() * D);
    if (R0 + R1 < R) 
        return false;

    //B1
    R0 = a(0) * abs(C.row(0)(1)) + a(1) * abs(C.row(1)(1)) + a(2) * abs(C.row(2)(1));
    R1 = b(1);
    R = abs(B1.transpose() * D);
    if (R0 + R1 < R) 
        return false;

    //B2
    R0 = a(0) * abs(C.row(0)(2)) + a(1) * abs(C.row(1)(2)) + a(2) * abs(C.row(2)(2));
    R1 = b(2);
    R = abs(B2.transpose() * D);
    if (R0 + R1 < R) 
        return false;

    //A0 X B0
    R0 = a(1) * abs(C.row(2)(0)) + a(2) * abs(C.row(1)(0));
    R1 = b(1) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(1));
    R = C.row(1)(0) * A2.transpose() * D;
    R -= C.row(2)(0) * A1.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A0 X B1
    R0 = a(1) * abs(C.row(2)(1)) + a(2) * abs(C.row(1)(1));
    R1 = b(0) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(0));
    R = C.row(1)(1) * A2.transpose() * D;
    R -= C.row(2)(1) * A1.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A0 X B2
    R0 = a(1) * abs(C.row(2)(2)) + a(2) * abs(C.row(1)(2));
    R1 = b(0) * abs(C.row(0)(1)) + b(1) * abs(C.row(0)(0));
    R = C.row(1)(2) * A2.transpose() * D;
    R -= C.row(2)(2) * A1.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A1 X B0
    R0 = a(0) * abs(C.row(2)(0)) + a(2) * abs(C.row(0)(0));
    R1 = b(1) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(1));
    R = C.row(2)(0) * A0.transpose() * D;
    R -= C.row(0)(0) * A2.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A1 X B1
    R0 = a(0) * abs(C.row(2)(1)) + a(2) * abs(C.row(0)(1));
    R1 = b(0) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(0));
    R = C.row(2)(1) * A0.transpose() * D;
    R -= C.row(0)(1) * A2.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A1 X B2
    R0 = a(0) * abs(C.row(2)(2)) + a(2) * abs(C.row(0)(2));
    R1 = b(0) * abs(C.row(1)(1)) + b(1) * abs(C.row(1)(0));
    R = C.row(2)(2) * A0.transpose() * D;
    R -= C.row(0)(2) * A2.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A2 X B0
    R0 = a(0) * abs(C.row(1)(0)) + a(1) * abs(C.row(0)(0));
    R1 = b(1) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(1));
    R = C.row(0)(0) * A1.transpose() * D;
    R -= C.row(1)(0) * A0.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A2 X B1
    R0 = a(0) * abs(C.row(1)(1)) + a(1) * abs(C.row(0)(1));
    R1 = b(0) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(0));
    R = C.row(0)(1) * A1.transpose() * D;
    R -= C.row(1)(1) * A0.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    //A2 X B2
    R0 = a(0) * abs(C.row(1)(2)) + a(1) * abs(C.row(0)(2));
    R1 = b(0) * abs(C.row(2)(1)) + b(1) * abs(C.row(2)(0));
    R = C.row(0)(2) * A1.transpose() * D;
    R -= C.row(1)(2) * A0.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) 
        return false;

    return true; //ready
}

