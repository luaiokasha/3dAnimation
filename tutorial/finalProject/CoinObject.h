#include "GameObject.h"

class CoinObject : public GameObject {
public:

	CoinObject(std::shared_ptr<cg3d::Movable> root, std::shared_ptr<cg3d::Material> material, BasicScene* BasicScene);
	void moveObject() override;
	void CollidedWithSnake() override;

private:
	double distance;
	double speed;
	int randomAxis;
};