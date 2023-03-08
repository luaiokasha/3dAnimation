#include "CoinObject.h"
#include <random>
using namespace cg3d;

CoinObject::CoinObject(std::shared_ptr<cg3d::Movable> root, std::shared_ptr<cg3d::Material> material, BasicScene* BasicScene) {
    
    scene = BasicScene;

    id = 3;
    distance = 0;
    speed = 0.0015;
    auto cube = Model::Create("cube", Mesh::Cylinder(), material);
    root->AddChild(cube);

    // create a random number generator engine
    std::random_device rd;
    std::mt19937 gen(rd());

    int min_num = -18;
    int max_num = 18;

    // create a distribution object for the range [min, max]
    std::uniform_int_distribution<> distr(min_num, max_num);

    // generate a random number between min_num and max_num
    cube->Translate(distr(gen), Movable::Axis::X);
    cube->Translate(distr(gen), Movable::Axis::Z);

    cube->Rotate(-140.0, Movable::Axis::Z);
    cube->Scale(0.5, Movable::Axis::X);

    model = cube;
    tree.init(model->GetMesh()->data[0].vertices, model->GetMesh()->data[0].faces);
  
    std::uniform_int_distribution<> dist(0, 1);

    randomAxis = dist(gen);
}

void CoinObject::moveObject()
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

void CoinObject::CollidedWithSnake() {

    // do something
    if (!scene->music) SoundHandler::getInstance().PlayOurSound("pickup");
    scene->AddScore(250);

    // destroy object + remove it from list
    // setting isHidden = true will destroy the object next frame in our code.
    model->isHidden = true;
}
