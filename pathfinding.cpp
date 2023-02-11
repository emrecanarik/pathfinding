#include <vector>
#include <queue>

struct Point
{
    int x;
    int y;
    
    Point(int x, int y)
    : x(x), y(y)
    {
    }
};

struct Node
{
    int index;
    int distanceToTarget;
    
    Node(int neighbor, int distanceToTarget)
    : index(neighbor), distanceToTarget(distanceToTarget)
    {
    }
    
    bool operator>(const Node& other) const
    {
        return distanceToTarget > other.distanceToTarget;
    }
};

bool isNeighborValid(const int neighborIndex,
                     const int currentIndex,
                     const int direction,
                     const int mapWidth,
                     const int mapSize);
bool isSamePoint(const Point& pointA, const Point& pointB);
int getIndex    (const int mapWidth, const Point point);
int getDistance (const int mapWidth, const int index, const int targetIndex);
Point getPoint  (const int mapWidth, const int index);


bool FindPath(const Point Start,
              const Point Target,
              const std::vector<int>& Map,
              const std::pair<int, int> MapDimensions,
              std::vector<int>& OutPath)
{
    const bool areStartAndTargetSameLocation = isSamePoint(Start, Target);
    
    if (areStartAndTargetSameLocation)
    {
        return true;
    }
    
    const int mapSize = MapDimensions.first * MapDimensions.second;
    const int mapWidth = MapDimensions.first;
    const int startIndex = getIndex(mapWidth, Start);
    const int targetIndex = getIndex(mapWidth, Target);
    
    std::vector<int> parentMap(mapSize);
    std::vector<int> distanceMap(mapSize, INT_MAX);
    
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    
    distanceMap[startIndex] = 0;
    
    const std::vector<int> directions =
    {
        1,          //right
        -1,         //left
        mapWidth,   //up
        -mapWidth   //down
    };
    
    Node start = Node(startIndex,
                      distanceMap[startIndex] +
                      getDistance(mapWidth, startIndex, targetIndex));
    
    pq.push(start);
    
    bool foundPath = false;
    
    while(!pq.empty())
    {
        int currentIndex = (pq.top()).index;
        pq.pop();
        
        for(const int& direction : directions)
        {
            const int neighborIndex = currentIndex + direction;
            
            //Avoid invalid cells
            if (!isNeighborValid(neighborIndex,
                                 currentIndex,
                                 direction,
                                 mapWidth,
                                 mapSize))
            {
                continue;
            }
            
            //Check if neighbor is an obstacle
            if (Map.at(neighborIndex) == 0)
            {
                continue;
            }
            
            //Check if we have a shorter path to neighbor
            if (distanceMap[neighborIndex] <= distanceMap[currentIndex] + 1)
            {
                continue;
            }
                        
            parentMap[neighborIndex] = currentIndex;
            distanceMap[neighborIndex] = distanceMap[currentIndex] + 1;
            
            if (neighborIndex == targetIndex)
            {
                foundPath = true;
                
                break;
            }
            
            //Add neighbor to queue to check
            Node node = Node(neighborIndex,
                             distanceMap[neighborIndex] +
                             getDistance(mapWidth, neighborIndex, targetIndex));
            pq.push(node);
            
        }
        
        if (foundPath)
        {
            break;
        }
    }
    
    //Check if algorithm can reach to target
    if (distanceMap[targetIndex] == INT_MAX)
    {
        return false;
    }
    
    //Go through the shortest path from target to start position
    int current = targetIndex;
    
    for (int i = distanceMap[targetIndex] - 1; i >= 0; i--)
    {
        OutPath.push_back(current);
        
        current = parentMap[current];
    }
    
    std::reverse(OutPath.begin(), OutPath.end());
    
    return true;
}

int getIndex(const int mapWidth, const Point point)
{
    return point.x + (point.y * mapWidth);
}

Point getPoint(const int mapWidth, const int index)
{
    Point point = Point(index % mapWidth,
                        index / mapWidth);
    
    return point;
}

//Calculate Manhattan Distance from index to targetIndex
int getDistance(const int mapWidth, const int index, const int targetIndex)
{
    const Point start = getPoint(mapWidth, index);
    const Point target = getPoint(mapWidth, targetIndex);
    
    return abs(start.x - target.x) + abs(start.y - target.y);
}

bool isSamePoint(const Point& pointA, const Point& pointB)
{
    return (pointA.x == pointB.x) && (pointA.y == pointB.y);
}

bool isNeighborValid(const int neighborIndex,
                     const int currentIndex,
                     const int direction,
                     const int mapWidth,
                     const int mapSize)
{
    if ((direction == 1  && (neighborIndex % mapWidth == 0)) ||
        (direction == -1 && (currentIndex   % mapWidth == 0)) ||
        (neighborIndex < 0) ||
        (neighborIndex >= mapSize))
    {
        return false;
    }
    
    return true;
}
