#include "stdafx.h"
#include "GameWorld.h"
#include "NavMesh.h"
#include <tge/drawers/DebugDrawer.h>
#include <tge/input/InputManager.h>

GameWorld::GameWorld()
{
	myNavMesh = nullptr;
}

GameWorld::~GameWorld() 
{
	delete myNavMesh;
}

void GameWorld::Init()  
{
	myNavMesh = new NavMesh();
	myNavMesh->Init("navmesh/navmesh.obj");
}

void GameWorld::Update(Tga::InputManager* anInputManager)
{
	if (anInputManager->IsKeyPressed(VK_LBUTTON) == true)
	{
		const auto& engine = *Tga::Engine::GetInstance();
		Tga::Vector2f mousePos = anInputManager->GetMousePosition();

		if (myStartPosition.LengthSqr() == 0 || (myStartPosition.LengthSqr() != 0 && myEndPosition.LengthSqr() != 0))
		{
			myStartPosition = mousePos;
			myStartPosition.y = engine.GetRenderSize().y - myStartPosition.y;
			myStartPosition -= renderOffset;
			myEndPosition = { 0.f, 0.f };
			if (myNavMesh->GetNodeIndexFromPoint(myStartPosition) < 0)
			{
				myStartPosition = { 0.f, 0.f };
			}
		}
		else if (myEndPosition.LengthSqr() == 0)
		{
			myEndPosition = mousePos;
			myEndPosition.y = engine.GetRenderSize().y - myEndPosition.y;
			myEndPosition -= renderOffset;
			if (myNavMesh->GetNodeIndexFromPoint(myEndPosition) < 0)
			{
				myEndPosition = { 0.f, 0.f };
			}
			else 
			{
				myPath = myNavMesh->FindShortestPath(myStartPosition, myEndPosition);
			}
		}
	}
}

void GameWorld::Render()
{
	const auto& engine = *Tga::Engine::GetInstance();
	Tga::DebugDrawer& debugDrawer = engine.GetDebugDrawer();

	myNavMesh->RenderNavmesh(debugDrawer);
	if (myStartPosition.Length() != 0 && myEndPosition.Length() != 0)
	{
		if (myPath.size() > 0) 
		{
			myNavMesh->RenderPortals(debugDrawer);
			debugDrawer.DrawLine(myStartPosition + renderOffset, myPath[0] + renderOffset, { 1.0f, 0.0f, 0.8f, 1.0f });
			for (int i = 0; i < myPath.size() - 1; ++i)
			{
				debugDrawer.DrawLine(myPath[i] + renderOffset, myPath[i + 1] + renderOffset, { 1.0f, 0.0f, 0.8f, 1.0f });
			}
			debugDrawer.DrawLine(myPath.back() + renderOffset, myEndPosition + renderOffset, { 1.0f, 0.0f, 0.8f, 1.0f });
		}
		else 
		{
			myNavMesh->RenderConnections(debugDrawer);
		}
	}
	else 
	{
		myNavMesh->RenderConnections(debugDrawer);
	}


	if (myStartPosition.Length() != 0)
	{
		debugDrawer.DrawCircle({ myStartPosition.myX + renderOffset.x, myStartPosition.myY + renderOffset.y }, 10.0f, Tga::Color(0.0f, 1.0f, 0.0f, 1.0f));
	}

	if (myEndPosition.Length() != 0)
	{
		debugDrawer.DrawCircle({ myEndPosition.myX + renderOffset.x, myEndPosition.myY + renderOffset.y }, 20.0f, Tga::Color(0.0f, 1.0f, 1.0f, 1.0f));
	}
}