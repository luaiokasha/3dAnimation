#include "BasicScene.h"
#include "Model.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "CamModel.h"
#include "IglMeshLoader.h"
#include "Movable.h"
#include <boundary_loop.h>
#include <map_vertices_to_circle.h>
#include "../../igl/harmonic.h"
#include "../../igl/per_vertex_normals.h"
#include "../../igl/read_triangle_mesh.h"
#include <imgui.h>
#include <stb_image.h>
#include <iostream>
#include <cstdlib>
#include <random>
#include "GameObject.h"
#include "BombObject.h"
#include "CoinObject.h"
#include "Snake.h"
using namespace cg3d;

#pragma region ImGui

void TextCentered(std::string text, float margin = 0)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);

    ImGui::Text(text.c_str());
}

void cursorCentered(std::string text, float margin = 0)
{
    auto windowWidth = ImGui::GetWindowSize().x;
    auto textWidth = ImGui::CalcTextSize(text.c_str()).x;

    ImGui::SetCursorPosX((windowWidth - textWidth) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);
}

void BasicScene::BuildImGui()

{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;

    ImGui::Begin("Menu", pOpen, flags);

    switch (gameState)
    {
    case GameState::StartMenu:
    {
        float window_width = 200;
        float window_height = 190;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Snake 3D", 30);
        cursorCentered("Start Game", 25);
        if (ImGui::Button("Start Game"))
        {
            gameState = GameState::Level1;
        }
        cursorCentered("Quit Game", 25);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::Level1:
    {
        float window_width = 400;
        float window_height = 380;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Level 1", 30);
        TextCentered("Controls:", 10);
        TextCentered("W - Up        1 - Third person camera", 10);
        TextCentered("S - Down      2 - First person camera", 10);
        TextCentered("A - Left      3 - Static camera      ", 10);
        TextCentered("D - Right     ESC - Pause game       ", 10);
        TextCentered("Q - imune                            ", 10);

        TextCentered("collect all the coins in time!", 30);
        TextCentered("You got " + std::to_string(gameDuration.count()) + " seconds to collect them!", 10);


        cursorCentered("Start Level", 10);
        if (ImGui::Button("Start Level"))
            if (!music) SoundHandler::getInstance().PlayOurSound("button");
            LoadLevel1();
        break;
    }
    case GameState::Level2:
    {
        float window_width = 400;
        float window_height = 250;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("Level 2", 30);

        TextCentered("collect all the coins and dodge the bombs", 30);

        cursorCentered("Start Level", 20);
        if (ImGui::Button("Start Level"))
            if (!music) SoundHandler::getInstance().PlayOurSound("button");
            LoadLevel2();
        break;
    }

    case GameState::MidLevel:
    {
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);

        std::string scoreStr = "Score: ";
        scoreStr = scoreStr + std::to_string(score);
        TextCentered(scoreStr.c_str(), 0);


            std::string timerStr = "Timer: ";
            auto remaining_time = gameDuration - (std::chrono::steady_clock::now() - gameTime);
            auto a = std::chrono::duration_cast<std::chrono::seconds>(remaining_time).count();
            timerStr = timerStr + std::to_string(a);
            TextCentered(timerStr.c_str(), 0);

        if (currentLevel == 2)
        {
            unsigned int color;
            if (isImune)
                color = IM_COL32(128, 128, 128, 255);
            else if (imunity==1)
                color = IM_COL32(0, 255, 0, 255);
            else
                color = IM_COL32(255, 0, 0, 255);

            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::Text("Boost");
            ImGui::PopStyleColor();
            ImGui::SameLine();
        }
    }
    case GameState::WinLevel:
    {
        float window_width = 300;
        float window_height = 300;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("You Win!", 30);
        std::string scoreStr = "Score: ";
        scoreStr = scoreStr + std::to_string(score);
        TextCentered(scoreStr.c_str(), 30);

        cursorCentered("Next Level", 20);
        if (ImGui::Button("Next Level"))
        {
                if (!music) SoundHandler::getInstance().PlayOurSound("button");
                currentLevel++;
                if (currentLevel == 2) {
                    gameState = GameState::Level2;
                    LoadLevel2();
                }
                paused = false;
        }
        cursorCentered("Quit Game", 20);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::AfterLevel:
    {
        float window_width = 300;
        float window_height = 300;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("GAME OVER", 30);
        std::string scoreStr = "Score: ";
        scoreStr = scoreStr + std::to_string(score);
        TextCentered(scoreStr.c_str(), 30);

        cursorCentered("Next Level", 20);
        if (ImGui::Button("Next Level"))
        {
            if (!music) SoundHandler::getInstance().PlayOurSound("button");
            currentLevel++;
            if (currentLevel == 2) {
                gameState = GameState::Level2;
                LoadLevel2();
            }
            paused = false;
        }
        cursorCentered("Quit Game", 20);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case GameState::Pause:
    {
        float window_width = 380;
        float window_height = 320;
        ImGui::SetWindowPos(ImVec2((DISPLAY_WIDTH - window_width) / 2, (DISPLAY_HEIGHT - window_height) / 2), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(window_width, window_height));

        TextCentered("PAUSED", 30);
        std::string scoreStr = "Current Score: ";
        scoreStr = scoreStr + std::to_string(score);
        TextCentered(scoreStr.c_str(), 30);
        TextCentered("Controls:", 10);
        TextCentered("W - Up        1 - Third person camera", 10);
        TextCentered("S - Down      2 - First person camera", 10);
        TextCentered("A - Left      3 - Static camera      ", 10);
        TextCentered("D - Right     ESC - Pause game       ", 10);

        auto restY = ImGui::GetCursorPosY();
        ImGui::SetCursorPosX(60);
        ImGui::SetCursorPosY(restY + 20);
        
        cursorCentered("Resume");
        ImGui::SetCursorPosY(restY + 20);
        ImGui::SetCursorPosX(260);
        ImGui::SetCursorPosY(restY + 20);
        if (ImGui::Button("Resume"))
        {
            gameState = GameState::MidLevel;
            paused = false;
        }

        cursorCentered("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    default:
    {
        break;
    }
    }
    

    ImGui::End();
    }




void BasicScene::GoToEndGamePanel() {
    currentLevel = -1;
    gameState = GameState::MidLevel;
    paused = true;
}

#pragma endregion

#pragma region Levels
//list of objects
std::list<GameObject*> gameObjectsLst;

void BasicScene::EndLevel() {
    gameState = GameState::MidLevel;
    paused = true;
}

void BasicScene::LoadLevel1() {
    gameTime = std::chrono::steady_clock::now();
    currentLevel = 1;
    
    // setup snake position
    
    // load objects
    auto coinMaterial{ std::make_shared<Material>("blank", program) };
    coinMaterial->AddTexture(0, "textures/gold.jpg", 2);
    for (int i = 0; i < 10; i++) {
        GameObject* coin = new CoinObject(root, coinMaterial, this);
        gameObjectsLst.push_back(coin);
    }

    // reset camera
    SetCamera(0);

    allowMove = false;
}

void BasicScene::LoadLevel2() {
    imunity = 1;
    for (GameObject* obj : gameObjectsLst) {
        obj->model->isHidden = true;
    }

    // load objects
   
    auto bombMaterial{ std::make_shared<Material>("blank", program) };
    bombMaterial->AddTexture(0, "textures/carbon.jpg", 2);
    GameObject* bomb1 = new BombObject(root, bombMaterial, this);
    GameObject* bomb2 = new BombObject(root, bombMaterial, this);
    GameObject* bomb3 = new BombObject(root, bombMaterial, this);
    GameObject* bomb4 = new BombObject(root, bombMaterial, this);
    GameObject* bomb5 = new BombObject(root, bombMaterial, this);
    GameObject* bomb6 = new BombObject(root, bombMaterial, this);
    GameObject* bomb7 = new BombObject(root, bombMaterial, this);
    GameObject* bomb8 = new BombObject(root, bombMaterial, this);
    gameObjectsLst.push_back(bomb1);
    gameObjectsLst.push_back(bomb2);
    gameObjectsLst.push_back(bomb3);
    gameObjectsLst.push_back(bomb4);
    gameObjectsLst.push_back(bomb5);
    gameObjectsLst.push_back(bomb6);
    gameObjectsLst.push_back(bomb7);
    gameObjectsLst.push_back(bomb8);

    auto coinMaterial{ std::make_shared<Material>("blank", program) };
    coinMaterial->AddTexture(0, "textures/gold.jpg", 2);
    for (int i = 0; i < 10; i++) {
        GameObject* coin = new CoinObject(root, coinMaterial, this);
        gameObjectsLst.push_back(coin);
    }

	daylight->AddTexture(0, "textures/cubemaps/Daylight Box2_", 3);
    allowMove = false;
    gameTime = std::chrono::steady_clock::now();
}



#pragma endregion


BasicScene::BasicScene(std::string name, Display* display) : SceneWithImGui(std::move(name), display)
{
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
}

void BasicScene::Init(float fov, int width, int height, float near2, float far2)
{
    DISPLAY_HEIGHT = height;
    DISPLAY_WIDTH = width;
    // create the root object that will contain all visible models
    root = Movable::Create("root");
    AddChild(root);

    daylight = std::make_shared<Material>("daylight", "shaders/cubemapShader");
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    root->AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    meshDataIndex = 0;
    
    // add snake here ////
    snake  = Snake(root, camList);
    snake.InitSnake(number_of_bones);
    //////////////////////

    SetupCameras(fov, width, height, near2, far2);
}
void BasicScene::SetupSnake()
{
    program = std::make_shared<Program>("shaders/basicShader");
    auto snakeMesh{ IglLoader::MeshFromFiles("snakeMesh", "data/snake5.obj") };
    auto snakeMaterial{ std::make_shared<Material>("blank", program) };
    snakeMaterial->AddTexture(0, "textures/snake.jpg", 2);
    snake = Model::Create("snake", snakeMesh, snakeMaterial);

    auto morphFunc = [&](Model* model, cg3d::Visitor* visitor) {
        return meshDataIndex;
    };
    /*autoSnake = AutoMorphingModel::Create(*snake, morphFunc);
    autoSnake->showWireframe = true;
    root->AddChild(autoSnake);*/


    // Joints
    // create first cylinder and add as child to root
    snakes.push_back(AutoMorphingModel::Create(*snake, morphFunc));
    snakes[0]->Scale(scaleFactor, Axis::Z);
    snakes[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    snakes[0]->RotateByDegree(90, Eigen::Vector3f(0, 1, 0));
    root->AddChild(snakes[0]);

    treeSnake.init(snakes[0]->GetMesh()->data[0].vertices, snakes[0]->GetMesh()->data[0].faces);

    // Textures
    SetupTextures(0);
    SetupJoins();

}

void BasicScene::SetupJoins() {
    auto morphFunc = [&](Model* model, cg3d::Visitor* visitor) {
        return meshDataIndex;
    };
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto snakeMesh{ IglLoader::MeshFromFiles("snakeMesh", "data/snake5.obj") };
    auto snakeMaterial{ std::make_shared<Material>("blank", program) };
    snakeMaterial->AddTexture(0, "textures/snake.jpg", 2);
    for (int i = 1; i < 16; i++) {
        snake = Model::Create("snake", snakeMesh, snakeMaterial);
        snakes.push_back(AutoMorphingModel::Create(*snake, morphFunc));
        snakes[i]->Scale(scaleFactor, Axis::Z);
        snakes[i]->Translate(1.6f * scaleFactor, Axis::Z);
        snakes[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        snakes[i - 1]->AddChild(snakes[i]);

        SetupTextures(i);

        snakes[i]->isHidden = true;
    }
}

void BasicScene::AddPart() {
    if (snakeLength > 15) return;

    snakes[snakeLength]->isHidden = false;
    snakeLength++;
}

void BasicScene::SetupTextures(int index)
{
    Eigen::MatrixXd textureCoords; //texture map

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    cg3d::MeshData snakeMeshData = snakes[index]->GetMesh()->data[0];

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

    auto mesh = snakes[index]->GetMeshList();
    mesh[0]->data.push_back({ V, F, snakeMeshData.vertexNormals, TC });
    snakes[index]->SetMeshList(mesh);
    snakes[index]->meshIndex = snakes[index]->meshIndex + 1;
    meshDataIndex = 1;

}
#pragma region Collision

bool BasicScene::IsColliding() {
    for (GameObject* curObj : gameObjectsLst) {
        // check only if close
        Eigen::Vector3f distance = snake.snake_bones[0]->GetTranslation() - curObj->model->GetTranslation();
        // the smaller the number after the <    the less accurate the collision check will be but faster.
        if (sqrt(pow(distance[0], 2) + pow(distance[1], 2) + pow(distance[2], 2)) < 3) {
            if (AreTreesColliding(snake.treeSnake, curObj->tree, snake.snake_bones[0], curObj->model)) {
                // collided with curObj
                curObj->CollidedWithSnake();
                return true;
            }
        }

    }
    return false;
}


bool BasicScene::AreTreesColliding(igl::AABB<Eigen::MatrixXd, 3>& firstTree, igl::AABB<Eigen::MatrixXd, 3>& secondTree, std::shared_ptr<cg3d::Model> firstModel, std::shared_ptr<cg3d::Model> secondModel)
{
    if (!AreBoxesColliding(firstTree, secondTree, firstModel, secondModel)) {
        return false;
    }
    else {
        // there is box collision, check if leaves or need to go deeper.
        // check if they are leaves
        if (firstTree.is_leaf() && secondTree.is_leaf()) return true;
        else if (firstTree.is_leaf()) {
            if (secondTree.m_left != nullptr) return AreTreesColliding(firstTree, *secondTree.m_left, firstModel, secondModel);
            if (secondTree.m_right != nullptr) return AreTreesColliding(firstTree, *secondTree.m_right, firstModel, secondModel);
        }
        else if (secondTree.is_leaf()) {
            if (firstTree.m_left != nullptr) return AreTreesColliding(*firstTree.m_left, secondTree, firstModel, secondModel);
            if (firstTree.m_right != nullptr) return AreTreesColliding(*firstTree.m_right, secondTree, firstModel, secondModel);
        }
        else {
            // both are not leaves, check all
            return (AreTreesColliding(*firstTree.m_left, *secondTree.m_left, firstModel, secondModel) || AreTreesColliding(*firstTree.m_left, *secondTree.m_right, firstModel, secondModel)
                || AreTreesColliding(*firstTree.m_right, *secondTree.m_right, firstModel, secondModel) || AreTreesColliding(*firstTree.m_right, *secondTree.m_left, firstModel, secondModel));
        }
    }

    return false;
}

bool BasicScene::AreBoxesColliding(igl::AABB<Eigen::MatrixXd, 3>& firstTree, igl::AABB<Eigen::MatrixXd, 3>& secondTree, std::shared_ptr<cg3d::Model> firstModel, std::shared_ptr<cg3d::Model> secondModel)
{
    Eigen::AlignedBox<double, 3>& firstBox = firstTree.m_box;
    Eigen::AlignedBox<double, 3>& secondBox = secondTree.m_box;

    Eigen::Matrix3d firstRotation = firstModel->GetRotation().cast<double>();
    Eigen::Matrix3d secondRotation = secondModel->GetRotation().cast<double>();

    // Setting up our variables...

    double a_0 = firstBox.sizes()[0] / 2;
    double a_1 = firstBox.sizes()[1] / 2;
    double a_2 = firstBox.sizes()[2] / 2;
    double b_0 = secondBox.sizes()[0] / 2;
    double b_1 = secondBox.sizes()[1] / 2;
    double b_2 = secondBox.sizes()[2] / 2;

    // all rotations
    Eigen::RowVector3d A_0 = firstRotation * Eigen::Vector3d(1, 0, 0);
    Eigen::RowVector3d A_1 = firstRotation * Eigen::Vector3d(0, 1, 0);
    Eigen::RowVector3d A_2 = firstRotation * Eigen::Vector3d(0, 0, 1);
    Eigen::RowVector3d B_0 = secondRotation * Eigen::Vector3d(1, 0, 0);
    Eigen::RowVector3d B_1 = secondRotation * Eigen::Vector3d(0, 1, 0);
    Eigen::RowVector3d B_2 = secondRotation * Eigen::Vector3d(0, 0, 1);

    //// all rotation to matrix
    Eigen::Matrix3d A;
    A << Eigen::RowVector3d(A_0[0], A_1[0], A_2[0]), Eigen::RowVector3d(A_0[1], A_1[1], A_2[1]), Eigen::RowVector3d(A_0[2], A_1[2], A_2[2]);
    Eigen::Matrix3d B;
    B << Eigen::RowVector3d(B_0[0], B_1[0], B_2[0]), Eigen::RowVector3d(B_0[1], B_1[1], B_2[1]), Eigen::RowVector3d(B_0[2], B_1[2], B_2[2]);

    // from equation
    Eigen::Matrix3d C = B * A.transpose();

    Eigen::Matrix4f firstTransform = firstModel->GetTransform().cast<float>();
    Eigen::Matrix4f secondTransform = secondModel->GetTransform().cast<float>();

    //// maybe we can do it without this calcs
    Eigen::Vector4f C_0_temp = firstTransform * Eigen::Vector4f(firstBox.center()[0], firstBox.center()[1], firstBox.center()[2], 1);
    Eigen::Vector4f C_1_temp = secondTransform * Eigen::Vector4f(secondBox.center()[0], secondBox.center()[1], secondBox.center()[2], 1);

    //// for equations
    Eigen::Vector3d D = Eigen::Vector3d(C_1_temp[0], C_1_temp[1], C_1_temp[2]) - Eigen::Vector3d(C_0_temp[0], C_0_temp[1], C_0_temp[2]);


    double first, second, sum;
    /// All 15 cases from the lectures
    first = (a_1 * abs(C(2, 0))) + (a_2 * abs(C(1, 0)));
    second = (b_1 * abs(C(0, 2))) + (b_2 * abs(C(1, 0)));
    sum = abs((C(1, 0) * A_2).dot(D) - (C(2, 0) * A_1).dot(D));
    if (sum > first + second) return false;

    first = (a_1 * abs(C(2, 1))) + (a_2 * abs(C(1, 1)));
    second = (b_0 * abs(C(0, 2))) + (b_2 * abs(C(0, 0)));
    sum = abs((C(1, 1) * A_2).dot(D) - (C(2, 1) * A_1).dot(D));
    if (sum > first + second) return false;

    first = (a_1 * abs(C(2, 2))) + (a_2 * abs(C(1, 2)));
    second = (b_0 * abs(C(0, 1))) + (b_1 * abs(C(0, 0)));
    sum = abs((C(1, 2) * A_2).dot(D) - (C(2, 2) * A_1).dot(D));
    if (sum > first + second) return false;

    first = (a_0 * abs(C(2, 0))) + (a_2 * abs(C(0, 0)));
    second = (b_1 * abs(C(1, 2))) + (b_2 * abs(C(1, 1)));
    sum = abs((C(2, 0) * A_0).dot(D) - (C(0, 0) * A_2).dot(D));
    if (sum > first + second) return false;

    first = (a_0 * abs(C(2, 1))) + (a_2 * abs(C(0, 1)));
    second = (b_0 * abs(C(1, 2))) + (b_2 * abs(C(1, 0)));
    sum = abs((C(2, 1) * A_0).dot(D) - (C(0, 1) * A_2).dot(D));
    if (sum > first + second) return false;

    first = (a_0 * abs(C(2, 2))) + (a_2 * abs(C(0, 2)));
    second = (b_0 * abs(C(1, 1))) + (b_1 * abs(C(1, 0)));
    sum = abs((C(2, 2) * A_0).dot(D) - (C(0, 2) * A_2).dot(D));
    if (sum > first + second) return false;

    first = (a_0 * abs(C(1, 0))) + (a_1 * abs(C(0, 0)));
    second = (b_1 * abs(C(2, 2))) + (b_2 * abs(C(2, 1)));
    sum = abs((C(0, 0) * A_1).dot(D) - (C(1, 0) * A_0).dot(D));
    if (sum > first + second) return false;

    first = (a_0 * abs(C(1, 1))) + (a_1 * abs(C(0, 1)));
    second = (b_0 * abs(C(2, 2))) + (b_2 * abs(C(2, 0)));
    sum = abs((C(0, 1) * A_1).dot(D) - (C(1, 1) * A_0).dot(D));
    if (sum > first + second) return false;

    first = (a_0 * abs(C(1, 2))) + (a_1 * abs(C(0, 2)));
    second = (b_0 * abs(C(2, 1))) + (b_1 * abs(C(2, 0)));
    sum = abs((C(0, 2) * A_1).dot(D) - (C(1, 2) * A_0).dot(D));
    if (sum > first + second) return false;

    first = (b_0 * abs(C(0, 0))) + (b_1 * abs(C(0, 1))) + (b_2 * abs(C(0, 2)));
    sum = abs(A_0.dot(D));
    if (sum > first + a_0) return false;

    first = (b_0 * abs(C(1, 0))) + (b_1 * abs(C(1, 1))) + (b_2 * abs(C(1, 2)));
    sum = abs(A_1.dot(D));
    if (sum > first + a_1) return false;

    first = (b_0 * abs(C(2, 0))) + (b_1 * abs(C(2, 1))) + (b_2 * abs(C(2, 2)));
    sum = abs(A_2.dot(D));
    if (sum > first + a_2) return false;

    first = (a_0 * abs(C(0, 0))) + (a_1 * abs(C(1, 0))) + (a_2 * abs(C(2, 0)));
    sum = abs(B_0.dot(D));
    if (sum > first + b_0) return false;

    first = (a_0 * abs(C(0, 1))) + (a_1 * abs(C(1, 1))) + (a_2 * abs(C(2, 1)));
    sum = abs(B_1.dot(D));
    if (sum > first + b_1) return false;

    first = (a_0 * abs(C(0, 2))) + (a_1 * abs(C(1, 2))) + (a_2 * abs(C(2, 2)));
    sum = abs(B_2.dot(D));
    if (sum > first + b_2) return false;

    return true;
}

#pragma endregion
#pragma region Input Callbacks


void BasicScene::KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key)
        {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_UP:
            snake.snake_bones[0]->RotateByDegree(8, Axis::X);
            if (snakeLength > 1)
                snake.snake_bones[1]->RotateByDegree(-8, Axis::X);
            for (int i = 2; i < snakeLength; i++) {
                snake.snake_bones[i]->RotateByDegree(-8 / i, Axis::X);
            }
            break;
        case GLFW_KEY_DOWN:
            snake.snake_bones[0]->RotateByDegree(-8, Axis::X);
            if (snakeLength > 1)
                snake.snake_bones[1]->RotateByDegree(8, Axis::X);
            for (int i = 2; i < snakeLength; i++) {
                snake.snake_bones[i]->RotateByDegree(8 / i, Axis::X);
            }
            break;
        case GLFW_KEY_LEFT:
            snake.snake_bones[0]->RotateByDegree(8, Axis::Y);
            if (snakeLength > 1)
                snake.snake_bones[1]->RotateByDegree(-8, Axis::Y);
            for (int i = 2; i < snakeLength; i++) {
                snake.snake_bones[i]->RotateByDegree(-8 / i, Axis::Y);
            }
            break;
        case GLFW_KEY_RIGHT:
            snake.snake_bones[0]->RotateByDegree(-8, Axis::Y);
            if (snakeLength > 1)
                snake.snake_bones[1]->RotateByDegree(8, Axis::Y);
            for (int i = 2; i < snakeLength; i++) {
                snake.snake_bones[i]->RotateByDegree(8 / i, Axis::Y);
            }
            break;
        case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.05f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.05f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.05f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.05f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.05f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.05f });
            break;
        case GLFW_KEY_Q:
            isImune = true;
            imunity--;
            break;
        case GLFW_KEY_1:
            SetCamera(0);
            break;
        case GLFW_KEY_2:
            SetCamera(1);
            break;
        case GLFW_KEY_3:
            SetCamera(2);
            break;
        case GLFW_KEY_4:
            score += 1000;
            gameState = GameState::MidLevel;
            paused = true;
            break;
        case GLFW_KEY_5:  
            break;
        case GLFW_KEY_SPACE:
            allowMove = true;
            prevTime = std::chrono::high_resolution_clock::now();
            break;
        }

    }
}

void BasicScene::MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
}

void BasicScene::ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
}

void BasicScene::CursorPosCallback(cg3d::Viewport* viewport, int xNew, int yNew, bool dragging, int* buttonState)
{
}

#pragma endregion

void BasicScene::SetupCameras(float fov, int width, int height, float near2, float far2)
{
    // create the camera objects
    camList.resize(camList.capacity());
    camList[0] = Camera::Create("camera0", fov, float(width) / float(height), near2, far2);
    
    auto program = std::make_shared<Program>("shaders/basicShader"); // TODO: TAL: replace with hard-coded basic program
    auto carbon = std::make_shared<Material>("carbon", program); // default material
    carbon->AddTexture(0, "textures/carbon.jpg", 2);
    auto camera1 = Camera::Create("camera1", fov, double(width) / height, near2, far2);
    auto model = ObjLoader::ModelFromObj("camera1", "data/camera.obj", carbon);
    model->isHidden = true;
    snake.snake_bones[0]->AddChild(camList[1] = CamModel::Create(*camera1, *model));
    auto camera2 = Camera::Create("camera2", fov, double(width) / height, near2, far2);
    auto model2 = ObjLoader::ModelFromObj("camera2", "data/camera.obj", carbon);
    model2->isHidden = true;
    snake.snake_bones[0]->AddChild(camList[2] = CamModel::Create(*camera2, *model2));

    camList[0]->Translate(50, Axis::Y);
    camList[0]->RotateByDegree(-90, Axis::X);
    camList[1]->Translate(-1, Axis::Z);
    camList[2]->Translate(20, Axis::Y);
    camList[2]->RotateByDegree(-90, Axis::X);

    camera = camList[0];
}

void BasicScene::SetCamera(int index)
{
    camera = camList[index];
    viewport->camera = camera;
}

void BasicScene::AddViewportCallback(Viewport* _viewport)
{
    viewport = _viewport;
    Scene::AddViewportCallback(viewport);
}

void BasicScene::FollowSnake()
{
    Eigen::Vector3f vec1 = Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f vec2;
    Eigen::Vector3f vec3 = Eigen::Vector3f(0, 0, 1);
    Eigen::Vector3f vec4;
    Eigen::Vector3f vec5 = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f vec6;

    for (int i = snakeLength - 1; i > 0; i--)
    {
        // X Axis
        vec2 = snake.snake_bones[i]->Tout.rotation() * Eigen::Vector3f(1, 0, 0);
        Eigen::Quaternionf quat = Eigen::Quaternionf::FromTwoVectors(vec2, vec1);
        quat = quat.slerp(0.99, Eigen::Quaternionf::Identity());
        snake.snake_bones[i]->Rotate(Eigen::Matrix3f(quat));
        // Y Axis
        vec4 = snake.snake_bones[i]->Tout.rotation() * Eigen::Vector3f(0, 0, 1);
        quat = Eigen::Quaternionf::FromTwoVectors(vec4, vec3);
        quat = quat.slerp(0.99, Eigen::Quaternionf::Identity());
        snake.snake_bones[i]->Rotate(Eigen::Matrix3f(quat));
        // Z Axis
        vec6 = snake.snake_bones[i]->Tout.rotation() * Eigen::Vector3f(0, 1, 0);
        quat = Eigen::Quaternionf::FromTwoVectors(vec6, vec5);
        quat = quat.slerp(0.99, Eigen::Quaternionf::Identity());
        snake.snake_bones[i]->Rotate(Eigen::Matrix3f(quat));
    }
}

Eigen::Vector3f BasicScene::getTipOfCyl(int cyl_index) {
    Eigen::Matrix4f linkTransform = snake.snake_bones[cyl_index]->GetAggregatedTransform();
    Eigen::Vector3f linkCenter = Eigen::Vector3f(linkTransform.col(3).x(), linkTransform.col(3).y(), linkTransform.col(3).z());
    return linkCenter + snake.snake_bones[cyl_index]->GetRotation() * Eigen::Vector3f(0, 0, CYL_LENGTH / 2);		  
}

Eigen::Vector3f BasicScene::getBaseOfCyl(int cyl_index) {
    // get base of cylinder
    Eigen::Matrix4f linkTransform = snake.snake_bones[cyl_index]->GetAggregatedTransform();
    Eigen::Vector3f linkCenter = Eigen::Vector3f(linkTransform.col(3).x(), linkTransform.col(3).y(), linkTransform.col(3).z());
    return linkCenter - snake.snake_bones[cyl_index]->GetRotation() * Eigen::Vector3f(0, 0, CYL_LENGTH / 2);
}		  

void BasicScene::Update(const Program& p, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    if (currentLevel == 1 && std::chrono::steady_clock::now() - gameTime > gameDuration)
    {
        EndLevel();
    }
    if (gameState == GameState::StartMenu) return;
    
    Scene::Update(p, proj, view, model);
	
    if (paused || !allowMove) return;	

    RemoveAndDestroyObjects();
	
    for (GameObject* var : gameObjectsLst)
    {
        var->moveObject();
	}		

    // how much time elapsed between the last frame and this one
    std::chrono::steady_clock::time_point endTime = std::chrono::high_resolution_clock::now();
    float deltaTime = std::chrono::duration<float, std::chrono::seconds::period>(endTime - prevTime).count();
    prevTime = endTime;

    auto system = snake.snake_bones[0]->GetRotation().transpose();
    snake.snake_bones[0]->TranslateInSystem(system, { 0 , 0, -2.0f * deltaTime});
    
    FollowSnake();
	
    currentUpdateAmount++;
    if (isImune == true) {
        // add timer and check
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - oldTime);
        if (elapsed.count() >= 3000) isImune = false;
    }

    if (currentUpdateAmount >= checkEveryXUpdates) {
        currentUpdateAmount = 0;
        IsColliding();
    }
}

#pragma region GameObjects
void BasicScene::appearingObject() {
    auto program = std::make_shared<Program>("shaders/basicShader"); // TODO: TAL: replace with hard-coded basic program
    //auto cubeMesh{ IglLoader::MeshFromFiles("cubeMesh", "data/cube.off") };
    auto cubeMaterial{ std::make_shared<Material>("blank", program) };
    cubeMaterial->AddTexture(0, "textures/grass.bmp", 2);
    auto cube = Model::Create("cube", Mesh::Cube(), cubeMaterial);
    root->AddChild(cube);

    std::random_device rd;
    std::mt19937 gen(rd());

    int min_num = -20;
    int max_num = 20;

    std::uniform_int_distribution<> distr(min_num, max_num);
    // generate a random number between min_num and max_num
    cube->Translate(distr(gen), Axis::X);
    cube->Translate(distr(gen), Axis::Z);
}

void BasicScene::RemoveAndDestroyObjects() {
													   
    for (auto it = gameObjectsLst.begin(); it != gameObjectsLst.end(); ) {
        if ((*it)->model->isHidden == true) {
            delete *it;
            it = gameObjectsLst.erase(it);
        }
        else {
            ++it;
        }
    }
}

void BasicScene::AddScore(int amount) {
    score += amount;
}

#pragma endregion
