

#include "Snake.h"
#include "BasicScene.h"

using namespace cg3d;
using namespace std;
using namespace Eigen;

Snake::Snake(std::shared_ptr<Movable> root, vector<std::shared_ptr<Camera>> camera_list)
{
    this->root = root;
    this->camera_list = camera_list;
}

void Snake::InitSnake(int num_of_bones)
{
    /*
    number_of_bones = num_of_bones;
    last_index = number_of_bones - 1;
    snake_length = bone_size * number_of_bones;


    // Snake Texture Material
    //auto program = std::make_shared<Program>("shaders/basicShader");
    //auto snake_material = std::make_shared<Material>("snake_material", program);
    //snake_material->AddTexture(0, "textures/snake.jpg", 2);


    // Snake Color Material
    auto snake_program = std::make_shared<Program>("shaders/phongShader");
    auto snake_material = std::make_shared<Material>("snake_material", snake_program);
    snake_material->program->name = "snake program";


    // Creating first bone mesh
    int i = 0;
    //snake_bones.push_back(ObjLoader::ModelFromObj("bone " + to_string(i), "data/zcylinder.obj", snake_material));
    snake_bones.push_back(ObjLoader::ModelFromObj("bone " + to_string(i), "data/zcylinder.obj", snake_material));
    root->AddChild(snake_bones[i]);
    snake_bones[first_index]->Translate({ 0,0,(bone_size / 2) });
    snake_bones[i]->SetCenter(Eigen::Vector3f(0, 0, -(bone_size / 2)));
    snake_bones[i]->showWireframe = false;
    snake_bones[i]->isHidden = true;
    i++;
    treeSnake.init(snake_bones[0]->GetMesh()->data[0].vertices, snake_bones[0]->GetMesh()->data[0].faces);
    // Creating the other bone meshes
    while (i < number_of_bones)
    {
        //snake_bones.push_back(ObjLoader::ModelFromObj("bone " + to_string(i), "data/zcylinder.obj", snake_material));
        snake_bones.push_back(ObjLoader::ModelFromObj("bone " + to_string(i), "data/zcylinder.obj", snake_material));
        snake_bones[i - 1]->AddChild(snake_bones[i]);
        snake_bones[i]->Translate(bone_size, Scene::Axis::Z);
        snake_bones[i]->SetCenter(Eigen::Vector3f(0, 0, -(bone_size / 2)));
        snake_bones[i]->showWireframe = false;
        snake_bones[i]->isHidden = true;
        i++;
    }
    SkinningInit();
}


void Snake::ShowSnake()
{
    if (skinning_enabled) {
        snake_body->isHidden = false;
    }
    else {
        for (int i = 0; i < number_of_bones; i++) {
            snake_bones[i]->isHidden = false;
        }
    }
    snake_head->isHidden = false;
    snake_tail->isHidden = false;
}

void Snake::HideSnake()
{
    if (skinning_enabled) {
        snake_body->isHidden = true;
    }
    else {
        for (int i = 0; i < number_of_bones; i++) {
            snake_bones[i]->isHidden = true;
        }
    }
    snake_head->isHidden = true;
    snake_tail->isHidden = true;
}

void Snake::ResetSnakePosition()
{
    std::vector<std::shared_ptr<cg3d::Model>> snake_bones = GetBones();
    root->RemoveChild(snake_bones[first_index]);
    snake_bones[first_index]->SetTransform(Matrix4f::Identity());
    root->AddChild(snake_bones[first_index]);

    int i = 1;
    while (i < number_of_bones)
    {
        snake_bones[i - 1]->RemoveChild(snake_bones[i]);
        snake_bones[i]->SetTransform(Matrix4f::Identity());

        snake_bones[i]->Translate(bone_size, Scene::Axis::Z);
        snake_bones[i]->SetCenter(Eigen::Vector3f(0, 0, -(bone_size / 2)));
        snake_bones[i - 1]->AddChild(snake_bones[i]);

        i++;
    }
    snake_bones[first_index]->Translate({ 0,0,(bone_size / 2) });

    Skinning();
}
*/
// Snake Movement
}
void Snake::SetupSnake()
{
    program = std::make_shared<Program>("shaders/basicShader");
    auto snakeMesh{ IglLoader::MeshFromFiles("snakeMesh", "data/snake5.obj") };
    auto snakeMaterial{ std::make_shared<Material>("blank", program) };
    snakeMaterial->AddTexture(0, "textures/snake.jpg", 2);
    snakemodel = Model::Create("snake", snakeMesh, snakeMaterial);

    auto morphFunc = [&](Model* model, cg3d::Visitor* visitor) {
        return meshDataIndex;
    };
    /*autoSnake = AutoMorphingModel::Create(*snake, morphFunc);
    autoSnake->showWireframe = true;
    root->AddChild(autoSnake);*/


    // Joints
    // create first cylinder and add as child to root
    snake_bones.push_back(AutoMorphingModel::Create(*snakemodel, morphFunc));
    snake_bones[0]->Scale(scaleFactor, Axis::Z);
    snake_bones[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    snake_bones[0]->RotateByDegree(90, Eigen::Vector3f(0, 1, 0));
    root->AddChild(snake_bones[0]);

    treeSnake.init(snake_bones[0]->GetMesh()->data[0].vertices, snake_bones[0]->GetMesh()->data[0].faces);

    // Textures
    SetupTextures(0);
    SetupJoins();

}

void Snake::SetupJoins() {
    auto morphFunc = [&](Model* model, cg3d::Visitor* visitor) {
        return meshDataIndex;
    };
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto snakeMesh{ IglLoader::MeshFromFiles("snakeMesh", "data/snake5.obj") };
    auto snakeMaterial{ std::make_shared<Material>("blank", program) };
    snakeMaterial->AddTexture(0, "textures/snake.jpg", 2);
    for (int i = 1; i < 16; i++) {
        snakemodel = Model::Create("snake", snakeMesh, snakeMaterial);
        snake_bones.push_back(AutoMorphingModel::Create(*snakemodel, morphFunc));
        snake_bones[i]->Scale(scaleFactor, Axis::Z);
        snake_bones[i]->Translate(1.6f * scaleFactor, Axis::Z);
        snake_bones[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        snake_bones[i - 1]->AddChild(snake_bones[i]);

        SetupTextures(i);

        snake_bones[i]->isHidden = true;
    }
}

void Snake::AddPart() {
    if (snakeLength > 15) return;

    snake_bones[snakeLength]->isHidden = false;
    snakeLength++;
}

void Snake::SetupTextures(int index)
{
    Eigen::MatrixXd textureCoords; //texture map

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    cg3d::MeshData snakeMeshData = snake_bones[index]->GetMesh()->data[0];

    int sizeOfVRows = snakeMeshData.vertices.rows();
    int sizeOfFRows = snakeMeshData.faces.rows();

    V.resize(snakeMeshData.vertices.rows(), snakeMeshData.vertices.cols());
    F.resize(snakeMeshData.faces.rows(), snakeMeshData.faces.cols());

    for (int i = 0; i < sizeOfVRows; i++) {
        V.row(i) = snakeMeshData.vertices.row(i);
        // need to do some changes here
    }

    for (int i = 0; i < sizeOfFRows; i++) {
        F.row(i) = snakeMeshData.faces.row(i);
        // need to do some changes here
    }

    Eigen::MatrixXd TC(V.rows(), 2);
    Eigen::VectorXd vec(V.rows());
    vec = V.col(0) + V.col(1) + V.col(2);
    vec /= 3;
    TC.col(0) = vec;
    TC.col(1) = V.col(0);
    TC *= 2;

    auto mesh = snake_bones[index]->GetMeshList();
    mesh[0]->data.push_back({ V, F, snakeMeshData.vertexNormals, TC });
    snake_bones[index]->SetMeshList(mesh);
    snake_bones[index]->meshIndex = snake_bones[index]->meshIndex + 1;
    meshDataIndex = 1;

}
void Snake::MoveUp()
{
    snake_bones[first_index]->Rotate(0.1f, Scene::Axis::X);
    snake_bones[first_index + 1]->Rotate(-0.1f, Scene::Axis::X);
}

void Snake::MoveDown()
{
    snake_bones[first_index]->Rotate(-0.1f, Scene::Axis::X);
    snake_bones[first_index + 1]->Rotate(0.1f, Scene::Axis::X);
}

void Snake::MoveLeft()
{
    snake_bones[first_index]->Rotate(0.1f, Scene::Axis::Y);
    snake_bones[first_index + 1]->Rotate(-0.1f, Scene::Axis::Y);
}

void Snake::MoveRight()
{
    snake_bones[first_index]->Rotate(-0.1f, Scene::Axis::Y);
    snake_bones[first_index + 1]->Rotate(0.1f, Scene::Axis::Y);
}

void Snake::CalculateWeight()
{
    int n = int(V.rows());
    W = Eigen::MatrixXd::Zero(n, number_of_bones + 1);

    float snake_length = V.colwise().maxCoeff()[2] - V.colwise().minCoeff()[2];
    float bone_length = snake_length / 16.f;

    double min_z = V.colwise().minCoeff()[2];

    for (int i = 0; i < n; i++) {
        double current_z = V.row(i)[2];
        for (int j = 0; j < number_of_bones + 1; j++) {
            if ((current_z >= min_z + bone_length * j) && (current_z <= min_z + bone_length * (j + 1))) {
                double dist = abs(current_z - (min_z + bone_length * j));
                W.row(i)[j] = (bone_length - dist) / bone_length;
                W.row(i)[j + 1] = 1 - W.row(i)[j];
                break;
            }
        }
    }
}


void Snake::SkinningInit()
{
    // Create snake mesh
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto material = std::make_shared<Material>("snake_material", program);
    material->program->name = "snake body program";
    auto snake_mesh{ IglLoader::MeshFromFiles("snake_igl", "data/snake1.obj") };
    snake_body = Model::Create("snake", snake_mesh, material);
    root->AddChild(snake_body);
    snake_body->isHidden = true;

    // Find transform
    Eigen::MatrixXd OV = snake_body->GetMeshList()[0]->data[0].vertices;
    Eigen::Vector3d translate = Eigen::Vector3d(0.0f, 0.0f, number_of_bones * (bone_size / 2));
    Eigen::Vector3d scale = Eigen::Vector3d(1.0f, 1.0f, number_of_bones);
    Eigen::Affine3d Tout{ Eigen::Affine3d::Identity() };
    Eigen::Affine3d Tin{ Eigen::Affine3d::Identity() };
    Tout.pretranslate(translate);
    Tin.scale(scale);
    Eigen::MatrixX4d transform = Tout.matrix() * Tin.matrix();


    // Find the new V
    V = Eigen::MatrixXd::Zero(OV.rows(), OV.cols());
    for (int i = 0; i < OV.rows(); i++) {
        Eigen::Vector4d OV_i = Eigen::Vector4d(OV.row(i).x(), OV.row(i).y(), OV.row(i).z(), 1);
        Eigen::Vector4d V_i = transform * OV_i;
        V.row(i) = Eigen::Vector3d(V_i[0], V_i[1], V_i[2]);
    }


    // Set Snake Mesh new data
    Eigen::MatrixXd VN;
    igl::per_vertex_normals(V, snake_body->GetMeshList()[0]->data[0].faces, VN);
    std::shared_ptr<cg3d::Mesh> new_mesh = std::make_shared<cg3d::Mesh>(
        snake_body->name,
        V,
        snake_body->GetMeshList()[0]->data[0].faces,
        VN,
        snake_body->GetMeshList()[0]->data[0].textureCoords);
    snake_body->SetMeshList({ new_mesh });
    snake_body->SetTransform(Eigen::Matrix4f::Identity());


    // Init C
    C.resize(number_of_bones + 1, 3);
    for (int i = 0; i < number_of_bones; i++)
    {
        C.row(i) = GetBonePosition(i, -1).cast<double>();
    }
    C.row(number_of_bones) = GetBonePosition(last_index, 1).cast<double>();


    // Init W
    CalculateWeight();


    // vQ - rotations of joints
    // vT - translation of joints
    vQ.resize(number_of_bones + 1, Eigen::Quaterniond::Identity());
    vT.resize(number_of_bones + 1);
}


void Snake::Skinning()
{
    if (!skinning_enabled) {
        return;
    }

    // Calculate vT
    for (int i = 0; i < number_of_bones; i++) {
        vT[i] = GetBonePosition(i, -1).cast<double>() - (Vector3d)C.row(i);
    }
    vT[number_of_bones] = GetBonePosition(last_index, 1).cast<double>() - (Vector3d)C.row(number_of_bones);

    // Calling DQS
    igl::dqs(V, W, vQ, vT, U);

    // Update mesh
    Eigen::MatrixXd VN;
    igl::per_vertex_normals(U, snake_body->GetMeshList()[0]->data[0].faces, VN);
    std::shared_ptr<cg3d::Mesh> new_mesh = std::make_shared<cg3d::Mesh>(
        snake_body->name,
        U,
        snake_body->GetMeshList()[0]->data[0].faces,
        VN,
        snake_body->GetMeshList()[0]->data[0].textureCoords);
    snake_body->SetMeshList({ new_mesh });
}


// Get the position of snake_bones[bone_id] 
// (-1) - start
// 0 - center
// 1 - end
Eigen::Vector3f Snake::GetBonePosition(int bone_id, int position)
{
    Eigen::Vector3f half_length = Eigen::Vector3f(0.f, 0.f, 0.8f);

    Eigen::Matrix4f bone_transform = snake_bones[bone_id]->GetAggregatedTransform();
    Eigen::Vector3f bone_center = Eigen::Vector3f(bone_transform.col(3).x(), bone_transform.col(3).y(), bone_transform.col(3).z());
    Eigen::Vector3f bone_requested_position = bone_center + snake_bones[bone_id]->GetRotation() * half_length * position;

    return bone_requested_position;
}