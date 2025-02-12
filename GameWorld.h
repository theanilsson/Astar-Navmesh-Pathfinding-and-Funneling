#pragma once
#include <tge/graphics/Camera.h>

namespace Tga 
{
	class InputManager;
}

class NavMesh;
class GameWorld
{
public:
	GameWorld();
	~GameWorld();
	void Init();

	void Update(Tga::InputManager* anInputManager);
	void Render();

	NavMesh* myNavMesh;
	Tga::Vector2f myStartPosition;
	Tga::Vector2f myEndPosition;
	std::vector<Tga::Vector2f> myPath;
};