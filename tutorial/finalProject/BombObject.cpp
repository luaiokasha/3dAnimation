#include "BombObject.h"
#include <random>
using namespace cg3d;

BombObject::BombObject(std::shared_ptr<cg3d::Movable> root, std::shared_ptr<cg3d::Material> material, BasicScene* BasicScene) {
    
    scene = BasicScene;

    id = 1;
    distance = 0;
    speed = 0.001;
    auto octahedron = Model::Create("Octahedron", Mesh::Cube(), material);
    root->AddChild(octahedron);


    std::random_device rd;
    std::mt19937 gen(rd());

    int min_num = -19;
    int max_num = 19;

    std::uniform_int_distribution<> distr(min_num, max_num);
    // generate a random number between min_num and max_num
    octahedron->Translate(distr(gen), Movable::Axis::X);
    octahedron->Translate(distr(gen), Movable::Axis::Z);

    Eigen::Vector3f snakePos = scene->snake.snake_bones[0]->GetTranslation();
    Eigen::Vector3f ourPos = octahedron->GetTranslation();
    Eigen::Vector3f res = ourPos - snakePos;
    if (sqrt((res[0] * res[0]) + (res[1] * res[1]) + (res[2] * res[2])) < 5) {
        octahedron->Translate(5.0, Movable::Axis::X);
        octahedron->Translate(5.0, Movable::Axis::Z);
    }

    octahedron->RotateByDegree(45, Movable::Axis::XZ);

    model = octahedron;
    tree.init(model->GetMesh()->data[0].vertices, model->GetMesh()->data[0].faces);

    std::uniform_int_distribution<> dist(0, 1);

    randomAxis = dist(gen);
}

void BombObject::moveObject()
{
    if (randomAxis == 0) {
        model->Translate(speed, Movable::Axis::Z);
    }
    else
    {
        model->Translate(speed, Movable::Axis::X);
    }
    distance = distance + speed;
    if (distance > 5 || distance < 0)
    {
        speed = -1 * speed;
    }
}

void BombObject::CollidedWithSnake() {

    // do something
    if (!scene->music) SoundHandler::getInstance().PlayOurSound("gameover");
    if (!scene->isImune) {
        scene->GoToEndGamePanel();
    }
    else {
        scene->isImune=false;
    }
    // setting isHidden will delete the object and remove from list next frame
    model->isHidden = true;

}