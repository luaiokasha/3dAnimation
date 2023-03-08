#include "GameObject.h"

class BombObject : public GameObject {
public:
	BombObject(std::shared_ptr<cg3d::Movable> root, std::shared_ptr<cg3d::Material> material, BasicScene* BasicScene);
	void moveObject() override;
	void CollidedWithSnake() override;

private:
	double distance;
	double speed;
	int randomAxis;
};