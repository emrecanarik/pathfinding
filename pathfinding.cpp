#include <vector>
#include <queue>
#include <tuple>

bool FindPath(std::pair<int, int> Start,
              std::pair<int, int> Target,
              const std::vector<int>& Map,
              std::pair<int, int> MapDimensions,
              std::vector<int>& OutPath)
{
    const bool areStartAndTargetSameLocation = (Start.first == Target.first) && (Start.second == Target.second);
    
    if(areStartAndTargetSameLocation)
    {
        return true;
    }
    
    auto getIndex = [MapDimensions](int x, int y) {
        return x + y * MapDimensions.first;
    };
    
    //Calculate index manhattan distance to target
    auto distance = [=](int index) -> int {
        
        //Get coordinates from index
        const int x = index % MapDimensions.first;
        const int y = index / MapDimensions.first;
        
        return abs(x - Target.first) + abs(y - Target.second);
    };
    
    const int mapSize = MapDimensions.first * MapDimensions.second;
    const int startPos = getIndex(Start.first, Start.second);
    const int targetPos = getIndex(Target.first, Target.second);
    
    int discovered = 0;
    
    std::vector<int> parentMap(mapSize);
    std::vector<int> distanceMap(mapSize, INT_MAX);
    
    // A* with tie breaking
    std::priority_queue<std::tuple<int, int, int>,
    std::vector<std::tuple<int, int, int>>,
    std::greater<std::tuple<int, int, int>>> pq;
    
    distanceMap[startPos] = 0;
    
    pq.push(std::make_tuple(0 + distance(startPos), 0, startPos));
    
    bool foundPath = false;
    
    while(!pq.empty())
    {
        int current = std::get<2>(pq.top());
        pq.pop();
        
        for(auto direction : {+1, -1, +MapDimensions.first, -MapDimensions.first})
        {
            int neighbour = current + direction;
            
            //Avoid invalid cells
            if((direction == 1 && (neighbour % MapDimensions.first == 0)) ||
               (direction == -1 && (current % MapDimensions.first == 0)))
            {
                continue;
            }
            
            if (0 <= neighbour &&
                neighbour < mapSize &&
                distanceMap[neighbour] > distanceMap[current] + 1 &&
                Map[neighbour] == 1)
            {
                parentMap[neighbour] = current;
                distanceMap[neighbour] = distanceMap[current] + 1;
                
                if (neighbour == targetPos)
                {
                    foundPath = true;
                    
                    break;
                }
                
                //Add neighbour to queue to check
                pq.push(std::make_tuple(distanceMap[neighbour] + distance(neighbour), ++discovered, neighbour));
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
