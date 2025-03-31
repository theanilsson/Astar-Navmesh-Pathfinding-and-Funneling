#pragma once
#include <tge/math/vector2.h>

namespace Tga 
{
	class InputManager;
}

class NavMesh;
class GameWorld
{
public:
	GameWorld();
	GameWorld(const GameWorld& anOtherEntity) = delete;
	GameWorld& operator=(const GameWorld& anOtherEntity) = delete;
	GameWorld(GameWorld&& anOtherEntity) = default;
	GameWorld& operator=(GameWorld&& anOtherEntity) = default;
	~GameWorld();
	void Init();

	void Update(Tga::InputManager* anInputManager);
	void Render();

	NavMesh* myNavMesh;
	Tga::Vector2f myStartPosition;
	Tga::Vector2f myEndPosition;
	std::vector<Tga::Vector2f> myPath;
};
