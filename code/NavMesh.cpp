#include "stdafx.h"
#include "NavMesh.h"
#include <tge/drawers/DebugDrawer.h>
#include <fstream>
#include <string>
#include <queue>

void NavMesh::Init(const char* aObjFile)
{
	myMeshData = LoadMesh(aObjFile);
	CreateNodes();
	CalculateConnections();
}

Mesh NavMesh::LoadMesh(const char* aObjFile)
{
	Mesh mesh;
	std::fstream meshLoader;
	meshLoader.open(aObjFile, std::ios_base::in);
	assert(meshLoader.is_open() == true && "Error! Couldn't open file!");

	while (meshLoader.eof() == false)
	{
		std::string reader;
		meshLoader >> reader;
		if (reader == "v")
		{
			float x, y, z;
			meshLoader >> x;
			meshLoader >> y;
			meshLoader >> z;
			x *= sizeMultiplier;
			y *= sizeMultiplier;
			z *= sizeMultiplier;
			mesh.vertices.push_back({ x, z });
		}
		else if (reader == "f")
		{
			int index1, index2, index3;
			int unusedUVIndex;
			char divider;
			meshLoader >> index1;
			meshLoader >> divider;
			meshLoader >> unusedUVIndex;

			meshLoader >> index2;
			meshLoader >> divider;
			meshLoader >> unusedUVIndex;

			meshLoader >> index3;
			meshLoader >> divider;
			meshLoader >> unusedUVIndex;

			mesh.indices.push_back(index1 - 1);
			mesh.indices.push_back(index2 - 1);
			mesh.indices.push_back(index3 - 1);
		}
	}

	return mesh;
}

void NavMesh::CreateNodes()
{
	for (int nodeIndex = 0; nodeIndex < myMeshData.indices.size(); nodeIndex += 3)
	{
		Node newNode;
		newNode.indices[0] = myMeshData.indices[nodeIndex + 0];
		newNode.indices[1] = myMeshData.indices[nodeIndex + 1];
		newNode.indices[2] = myMeshData.indices[nodeIndex + 2];
		newNode.center = (myMeshData.vertices[newNode.indices[0]] + myMeshData.vertices[newNode.indices[1]] + myMeshData.vertices[newNode.indices[2]]) / 3.f;
		newNode.state = eNodeState::Unvisited;
		myNodes.push_back(newNode);
	}
}

void NavMesh::CalculateConnections()
{
	for (int i = 1; i < myNodes.size() - 1; i++)
	{
		if (myNodes[i].connections.size() == 3)
			continue;

		for (int j = i + 1; j < myNodes.size(); j++)
		{
			if (myNodes[j].connections.size() < 3 && AreConnected(i, j))
			{
				myNodes[i].connections.push_back(j);
				myNodes[j].connections.push_back(i);

				if (myNodes[i].connections.size() == 3)
					break;
			}
		}
	}
}

inline bool NavMesh::AreConnected(const int aFirstIndex, const int aSecondIndex)
{
	int numberOfEqualVertices = 0;
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[0]] == myMeshData.vertices[myNodes[aSecondIndex].indices[0]])
	{
		numberOfEqualVertices++;
	}
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[0]] == myMeshData.vertices[myNodes[aSecondIndex].indices[1]])
	{
		numberOfEqualVertices++;
	}
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[0]] == myMeshData.vertices[myNodes[aSecondIndex].indices[2]])
	{
		numberOfEqualVertices++;
	}

	if (myMeshData.vertices[myNodes[aFirstIndex].indices[1]] == myMeshData.vertices[myNodes[aSecondIndex].indices[0]])
	{
		numberOfEqualVertices++;
	}
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[1]] == myMeshData.vertices[myNodes[aSecondIndex].indices[1]])
	{
		numberOfEqualVertices++;
	}
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[1]] == myMeshData.vertices[myNodes[aSecondIndex].indices[2]])
	{
		numberOfEqualVertices++;
	}

	if (myMeshData.vertices[myNodes[aFirstIndex].indices[2]] == myMeshData.vertices[myNodes[aSecondIndex].indices[0]])
	{
		numberOfEqualVertices++;
	}
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[2]] == myMeshData.vertices[myNodes[aSecondIndex].indices[1]])
	{
		numberOfEqualVertices++;
	}
	if (myMeshData.vertices[myNodes[aFirstIndex].indices[2]] == myMeshData.vertices[myNodes[aSecondIndex].indices[2]])
	{
		numberOfEqualVertices++;
	}

	return numberOfEqualVertices >= 2;
}

std::vector<Tga::Vector2f> NavMesh::FindShortestPath(const Tga::Vector2f& aStart, const Tga::Vector2f& anEnd)
{
	std::vector<int> cameFrom(myNodes.size(), -1);
	std::vector<float> costSoFar(myNodes.size(), std::numeric_limits<float>::infinity());
	std::priority_queue<CostAndIndex> frontier;

	int startIndex = GetNodeIndexFromPoint(aStart);
	int endIndex = GetNodeIndexFromPoint(anEnd);
	if (startIndex == endIndex)
	{
		std::vector<Tga::Vector2f> outPath;
		outPath.push_back(anEnd);
		mySavedPortals.clear();
		return outPath;
	}
	frontier.push({ 0.0f, startIndex });
	myNodes[startIndex].state = eNodeState::Closed;
	for (int connectedIndex : myNodes[startIndex].connections)
	{
		myNodes[connectedIndex].state = eNodeState::Open;
	}
	cameFrom[startIndex] = startIndex;
	costSoFar[startIndex] = 0.0f;

	while (!frontier.empty())
	{
		CostAndIndex currentLocation = frontier.top();
		frontier.pop();
		myNodes[currentLocation.index].state = eNodeState::Closed;

		// Check if the algorithm found a valid path with the current frontier node
		if (currentLocation.index == endIndex)
		{
			// Construct path data
			std::vector<Node*> path;
			int current = cameFrom[endIndex];
			path.push_back(&myNodes[endIndex]);
			while (current != startIndex)
			{
				path.push_back(&myNodes[current]);
				current = cameFrom[current];
			}
			path.push_back(&myNodes[startIndex]);
			std::reverse(path.begin(), path.end());


			// Construct portals for path funneling
			std::vector<Edge> portals;
			portals.reserve(path.size() * 3 + 1);
			portals.push_back({ aStart, aStart });
			for (size_t i = 1; i < path.size() - 1; i++)
			{
				Node* currentNode = path[i];
				Node* nextNode = path[i + 1];

				for (int currentIndex = 0; currentIndex < 3; currentIndex++)
				{
					for (int nextIndex = 0; nextIndex < 3; nextIndex++)
					{
						Tga::Vector2f currentVertex1 = myMeshData.vertices[currentNode->indices[currentIndex]];
						Tga::Vector2f currentVertex2 = myMeshData.vertices[currentNode->indices[(currentIndex + 1) % 3]];
						Tga::Vector2f nextVertex1 = myMeshData.vertices[nextNode->indices[nextIndex]];
						Tga::Vector2f nextVertex2 = myMeshData.vertices[nextNode->indices[(nextIndex + 1) % 3]];

						if ((currentVertex1 == nextVertex1 && currentVertex2 == nextVertex2) || (currentVertex1 == nextVertex2 && currentVertex2 == nextVertex1))
						{
							portals.push_back({ currentVertex1, currentVertex2 });
						}
					}
				}
			}
			portals.push_back({ anEnd, anEnd });


			for (auto& node : myNodes)
			{
				node.state = eNodeState::Unvisited;
			}
			mySavedPortals = portals;
			std::vector<Tga::Vector2f> optimizedPath;
			FunnelPath(portals, optimizedPath);
			return optimizedPath;
		}

		// Traverse the current frontier node
		for (int next : myNodes[currentLocation.index].connections)
		{
			if (myNodes[next].state != eNodeState::Open)
				continue;

			float newCost = costSoFar[currentLocation.index] + (std::abs(myNodes[next].center.x - myNodes[currentLocation.index].center.x) + std::abs(myNodes[next].center.y - myNodes[currentLocation.index].center.y));
			if (costSoFar[next] == std::numeric_limits<float>::infinity() || newCost < costSoFar[next])
			{
				costSoFar[next] = newCost;
				float priority = newCost + GetHeuristic(myNodes[next].center, myNodes[endIndex].center);
				frontier.push({ priority, next });
				for (int connectedIndex : myNodes[next].connections)
				{
					if (myNodes[connectedIndex].state == eNodeState::Unvisited)
					{
						myNodes[connectedIndex].state = eNodeState::Open;
					}
				}
				cameFrom[next] = currentLocation.index;
			}
		}
	}

	for (auto& node : myNodes)
	{
		node.state = eNodeState::Unvisited;
	}
	return std::vector<Tga::Vector2f>();
}

int NavMesh::GetNodeIndexFromPoint(const Tga::Vector2f& aPoint) const
{
	for (int i = 0; i < myNodes.size(); i++)
	{
		const Tga::Vector2f& pointA = myMeshData.vertices[myNodes[i].indices[0]];
		const Tga::Vector2f& pointB = myMeshData.vertices[myNodes[i].indices[1]];
		const Tga::Vector2f& pointC = myMeshData.vertices[myNodes[i].indices[2]];

		float minX = std::min({ pointA.x, pointB.x, pointC.x });
		float maxX = std::max({ pointA.x, pointB.x, pointC.x });
		float minY = std::min({ pointA.y, pointB.y, pointC.y });
		float maxY = std::max({ pointA.y, pointB.y, pointC.y });
		if (aPoint.x < minX || aPoint.x > maxX || aPoint.y < minY || aPoint.y > maxY) 
			continue;

		float denominator = (pointB.y - pointC.y) * (pointA.x - pointC.x) + (pointC.x - pointB.x) * (pointA.y - pointC.y);
		if (denominator == 0.0f) 
			continue; // Degenerate triangle

		float w1 = ((pointB.y - pointC.y) * (aPoint.x - pointC.x) + (pointC.x - pointB.x) * (aPoint.y - pointC.y)) / denominator;
		float w2 = ((pointC.y - pointA.y) * (aPoint.x - pointC.x) + (pointA.x - pointC.x) * (aPoint.y - pointC.y)) / denominator;
		float w3 = 1.0f - w1 - w2;
		if (w1 >= 0 && w2 >= 0 && w3 >= 0) 
			return i; // Found node
	}

	return -1;
}

void NavMesh::FunnelPath(const std::vector<Edge>& somePortals, std::vector<Tga::Vector2f>& outPath)
{
	Tga::Vector2f funnelApex = somePortals[0].firstPoint;
	Tga::Vector2f funnelLeft;
	Tga::Vector2f funnelRight;
	Tga::Vector2f firstMidPoint = FindMidpoint(somePortals[1].firstPoint, somePortals[1].secondPoint);
	if (IsToTheRightOfLine(funnelApex, firstMidPoint, somePortals[1].firstPoint))
	{
		funnelRight = somePortals[1].firstPoint;
		funnelLeft = somePortals[1].secondPoint;
	}
	else
	{
		funnelRight = somePortals[1].secondPoint;
		funnelLeft = somePortals[1].firstPoint;
	}

	int leftIndex = 1, rightIndex = 1;
	outPath.push_back(funnelApex);

	for (int i = 2; i < somePortals.size(); ++i)
	{
		if (somePortals[i].firstPoint == funnelApex || somePortals[i].secondPoint == funnelApex)
			continue;

		Tga::Vector2f portalLeft;
		Tga::Vector2f portalRight;
		Tga::Vector2f nextMidPoint = FindMidpoint(somePortals[i].firstPoint, somePortals[i].secondPoint);
		if (IsToTheRightOfLine(funnelApex, nextMidPoint, somePortals[i].firstPoint))
		{
			portalRight = somePortals[i].firstPoint;
			portalLeft = somePortals[i].secondPoint;
		}
		else 
		{
			portalRight = somePortals[i].secondPoint;
			portalLeft = somePortals[i].firstPoint;
		}
		if (funnelLeft == funnelApex) 
		{
			funnelLeft = portalLeft;
			leftIndex = i;
		}
		if (funnelRight == funnelApex) 
		{
			funnelRight = portalRight;
			rightIndex = i;
		}

		if (IsToTheLeftOfLine(funnelApex, portalLeft, funnelLeft)) 
		{
			if (IsToTheRightOfLine(funnelApex, funnelRight, portalLeft)) 
			{
				// Set new funnel apex using right vertex
				funnelApex = funnelRight;
				funnelLeft = funnelApex;
				i = rightIndex;
				outPath.push_back(funnelApex);
			}
			else 
			{
				// Narrow left side of funnel
				funnelLeft = portalLeft;
				leftIndex = i;
			}
		}
		if (IsToTheRightOfLine(funnelApex, portalRight, funnelRight)) 
		{
			if (IsToTheLeftOfLine(funnelApex, funnelLeft, portalRight))
			{
				// Set new funnel apex using left vertex
				funnelApex = funnelLeft;
				funnelRight = funnelApex;
				i = leftIndex;
				outPath.push_back(funnelApex);
			}
			else
			{
				// Narrow right side of funnel
				funnelRight = portalRight;
				rightIndex = i;
			}
		}
	}

	outPath.push_back(somePortals.back().firstPoint);
}

void NavMesh::RenderNavmesh(Tga::DebugDrawer& aDebugDrawer)
{
	const Tga::Vector2f offset = renderOffset;
	for (int nodeIndex = 0; nodeIndex < myMeshData.indices.size(); nodeIndex += 3)
	{
		int index1 = myMeshData.indices[nodeIndex + 0];
		int index2 = myMeshData.indices[nodeIndex + 1];
		int index3 = myMeshData.indices[nodeIndex + 2];

		Tga::Vector2f vertex1 = { myMeshData.vertices[index1].myX, myMeshData.vertices[index1].myY };
		Tga::Vector2f vertex2 = { myMeshData.vertices[index2].myX, myMeshData.vertices[index2].myY };
		Tga::Vector2f vertex3 = { myMeshData.vertices[index3].myX, myMeshData.vertices[index3].myY };

		aDebugDrawer.DrawLine(vertex1 + offset, vertex2 + offset);
		aDebugDrawer.DrawLine(vertex1 + offset, vertex3 + offset);
		aDebugDrawer.DrawLine(vertex2 + offset, vertex3 + offset);
	}
}

void NavMesh::RenderConnections(Tga::DebugDrawer& aDebugDrawer)
{
	const Tga::Vector2f offset = renderOffset;
	for (int nodeIndex = 0; nodeIndex < myNodes.size(); nodeIndex++)
	{
		const Node& currentNode = myNodes[nodeIndex];
		Tga::Vector2f from = { currentNode.center.myX, currentNode.center.myY };
		for (int connectionIndex = 0; connectionIndex < currentNode.connections.size(); connectionIndex++)
		{
			const Node& connectedNode = myNodes[currentNode.connections[connectionIndex]];
			Tga::Vector2f to = { connectedNode.center.myX, connectedNode.center.myY };

			aDebugDrawer.DrawLine(from + offset, to + offset, connectionsColor);
		}
	}
}

void NavMesh::RenderPortals(Tga::DebugDrawer& aDebugDrawer)
{
	const Tga::Vector2f offset = renderOffset;
	auto color = Tga::Color(0.0f, 0.5f, 0.8f, 1.0f);
	for (auto portal : mySavedPortals)
	{
		aDebugDrawer.DrawLine(portal.firstPoint + offset, portal.secondPoint + offset, color);
		color.myR *= 0.9f;
		color.myG += 10 % 2 * 0.9f;
	}
}