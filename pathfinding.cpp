#include <vector>
#include <queue>
#include <tuple>

struct Point
{
    int x;
    int y;
};

struct Something
{
    int x;
    int y;
    int z;
};

int getIndex(int mapWidth, Point point)
{
    return point.x + (point.y * mapWidth);
}

Point getPoint(int mapWidth, int index)
{
    Point point;
    
    point.x = index % mapWidth;
    point.y = index / mapWidth;
    
    return point;
}

//Calculate Manhattan Distance from index to targetIndex
int getDistance(int mapWidth, int index, int targetIndex)
{
    const Point start = getPoint(mapWidth, index);
    const Point target = getPoint(mapWidth, targetIndex);
    
    return abs(start.x - target.x) + abs(start.y - target.y);
}

bool isSamePoint(const Point& pointA, const Point& pointB)
{
    return (pointA.x == pointB.x) && (pointA.y == pointB.y);
}

bool FindPath(const Point Start,
              const Point Target,
              const std::vector<int>& Map,
              const std::pair<int, int> MapDimensions,
              std::vector<int>& OutPath)
{
    const bool areStartAndTargetSameLocation = isSamePoint(Start, Target);
    
    if(areStartAndTargetSameLocation)
    {
        return true;
    }
    
    const int mapSize = MapDimensions.first * MapDimensions.second;
    const int mapWidth = MapDimensions.first;
    const int startPos = getIndex(mapWidth, Start);
    const int targetPos = getIndex(mapWidth, Target);
    
    int discovered = 0;
    
    std::vector<int> parentMap(mapSize);
    std::vector<int> distanceMap(mapSize, INT_MAX);
    
    // A* with tie breaking
    std::priority_queue<std::tuple<int, int, int>,
                        std::vector<std::tuple<int, int, int>>,
                        std::greater<std::tuple<int, int, int>>> pq;
    
    distanceMap[startPos] = 0;
    
    //directions for right, left, up and down
    const std::vector<int> directions = {1, -1, mapWidth, -mapWidth};
    
    pq.push(std::make_tuple(0 + getDistance(mapWidth, startPos, targetPos), 0, startPos));
    
    bool foundPath = false;
    
    while(!pq.empty())
    {
        int current = std::get<2>(pq.top());
        pq.pop();
        
        for(auto direction : directions)
        {
            const int neighbour = current + direction;
            
            //Avoid invalid cells
            if((direction == 1 && (neighbour % mapWidth == 0)) ||
               (direction == -1 && (current % mapWidth == 0)))
            {
                continue;
            }
            
            if (0 <= neighbour &&
                neighbour < mapSize &&
                distanceMap[neighbour] > distanceMap[current] + 1 &&
                Map.at(neighbour) == 1)
            {
                parentMap[neighbour] = current;
                distanceMap[neighbour] = distanceMap[current] + 1;
                
                if (neighbour == targetPos)
                {
                    foundPath = true;
                    
                    break;
                }
                
                //Add neighbour to queue to check
                pq.push(std::make_tuple(distanceMap[neighbour] + getDistance(mapWidth, neighbour, targetPos),
                                        ++discovered,
                                        neighbour));
            }
        }
        
        if (foundPath)
        {
            break;
        }
    }
    
    //Check if algorithm can reach to target
    if (distanceMap[targetPos] == INT_MAX)
    {
        return false;
    }
    
    //Go through the shortest path from target to start position
    int current = targetPos;
    
    for (int i = distanceMap[targetPos] - 1; i >= 0; i--)
    {
        OutPath.push_back(current);
        
        current = parentMap[current];
    }
    
    std::reverse(OutPath.begin(), OutPath.end());
    
    return true;
}