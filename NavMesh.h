#pragma once
#include <vector>
#include <array>

namespace Tga 
{
	class DebugDrawer;
}

enum class eNodeState
{
	Unvisited,
	Open,
	Closed
};

struct Edge 
{
	Tga::Vector2f firstPoint;
	Tga::Vector2f secondPoint;
};

struct Node
{
	Tga::Vector2f center;
	std::vector<int> connections;
	std::array<int, 3> indices;
	eNodeState state;
};

struct Mesh
{
	std::vector<Tga::Vector2f> vertices;
	std::vector<int> indices;
};

struct CostAndIndex
{
	float cost;
	int index;
	bool operator<(const CostAndIndex& anOtherNodeCI) const
	{
		return cost > anOtherNodeCI.cost;
	}
};

inline constexpr Tga::Vector2f renderOffset(200.0f, 50.0f);
inline constexpr int sizeMultiplier = 700;
inline const Tga::Color connectionsColor(0.3f, 0.3f, 0.f, 0.5f);

constexpr float AbsF(float x) 
{
	return x < 0 ? -x : x;
}

class NavMesh
{
public:
	// Init stage functions //
	NavMesh() = default;
	~NavMesh() = default;
	void Init(const char* aObjFile);

	// Pathfinding functions //
	std::vector<Tga::Vector2f> FindShortestPath(const Tga::Vector2f& aStart, const Tga::Vector2f& anEnd);
	int GetNodeIndexFromPoint(const Tga::Vector2f& aPoint) const;

	// Render functions //
	void RenderNavmesh(Tga::DebugDrawer& aDebugDrawer);
	void RenderConnections(Tga::DebugDrawer& aDebugDrawer);
	void RenderPortals(Tga::DebugDrawer& aDebugDrawer);

private:
	// Init functions //
	Mesh LoadMesh(const char* aObjFile);
	void CreateNodes();
	void CalculateConnections();
	inline bool AreConnected(const int aFirstIndex, const int aSecondIndex);

	// Pathfinding functions //
	void FunnelPath(const std::vector<Edge>& somePortals, std::vector<Tga::Vector2f>& outPath);
	constexpr float GetHeuristic(const Tga::Vector2f& aPathNode, const Tga::Vector2f& anEndNode) const
	{
		return AbsF(aPathNode.x - anEndNode.x) + AbsF(aPathNode.y - anEndNode.y);
	}
	constexpr Tga::Vector2f FindMidpoint(const Tga::Vector2f& aFirstPoint, const Tga::Vector2f& aSecondPoint) const
	{
		return Tga::Vector2f((aFirstPoint.x + aSecondPoint.x) / 2.0f, (aFirstPoint.y + aSecondPoint.y) / 2.0f);
	}
	constexpr bool IsToTheLeftOfLine(const Tga::Vector2f& aLineStart, const Tga::Vector2f& aLineEnd, const Tga::Vector2f& aPoint) const
	{
		return ((aLineStart.x - aPoint.x) * (aLineEnd.y - aPoint.y) - (aLineEnd.x - aPoint.x) * (aLineStart.y - aPoint.y)) > 0.0f;
	}
	constexpr bool IsToTheRightOfLine(const Tga::Vector2f& aLineStart, const Tga::Vector2f& aLineEnd, const Tga::Vector2f& aPoint) const
	{
		return ((aLineStart.x - aPoint.x) * (aLineEnd.y - aPoint.y) - (aLineEnd.x - aPoint.x) * (aLineStart.y - aPoint.y)) < 0.0f;
	}

	Mesh myMeshData;
	std::vector<Node> myNodes;
	std::vector<Edge> mySavedPortals;
};