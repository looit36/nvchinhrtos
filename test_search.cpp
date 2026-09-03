#include <iostream>
#include <string>
#include <vector>
#include "lib/maze/include/MazeLib/Maze.h"
#include "lib/maze/include/MazeLib/SearchAlgorithm.h"
using namespace std;
int main() {
    string mapHex = "MAP:0010D6C6C44444444446E93A8000000000029553800000000002C444000000000002800000000000000280000000000000028000000000000002800000000000000280000000000000028000000000000002800000000000000280000000000000028000000000000002800000000000000280000000000000029111111111111113";
    mapHex = mapHex.substr(4);
    int startX = (mapHex[0] >= 'A') ? (mapHex[0] - 'A' + 10) : (mapHex[0] - '0');
    int startY = (mapHex[1] >= 'A') ? (mapHex[1] - 'A' + 10) : (mapHex[1] - '0');
    int goalX  = (mapHex[2] >= 'A') ? (mapHex[2] - 'A' + 10) : (mapHex[2] - '0');
    int goalY  = (mapHex[3] >= 'A') ? (mapHex[3] - 'A' + 10) : (mapHex[3] - '0');
    MazeLib::Positions goals; goals.push_back(MazeLib::Position(goalX, goalY));
    MazeLib::Maze maze(goals, MazeLib::Position(startX, startY)); maze.reset();
    int idx = 4;
    for (int x = 0; x < 16; x++) {
        for (int y = 0; y < 16; y++) {
            char c = mapHex[idx++]; uint8_t val = 0;
            if (c >= '0' && c <= '9') val = c - '0'; else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::East, val & 1, false);
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::North, val & 2, false);
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::West, val & 4, false);
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::South, val & 8, false);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::East, true);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::North, true);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::West, true);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::South, true);
        }
    }
    maze.updateWall(MazeLib::Position(startX, startY), MazeLib::Direction::East, true, true);
    maze.updateWall(MazeLib::Position(startX, startY), MazeLib::Direction::West, true, true);
    maze.updateWall(MazeLib::Position(startX, startY), MazeLib::Direction::South, true, true);
    MazeLib::SearchAlgorithm searcher(maze); MazeLib::Directions shortestPath;
    if (searcher.calcShortestDirections(shortestPath, true)) {
        string keriseStr = "s";
        MazeLib::Direction prevDir = shortestPath[0];
        for (size_t i = 1; i < shortestPath.size(); ++i) {
            MazeLib::Direction nextDir = shortestPath[i];
            int diff = (nextDir - prevDir + 8) % 8;
            if (diff == 0) keriseStr += "S"; else if (diff == 6) keriseStr += "R"; else if (diff == 2) keriseStr += "L";
            prevDir = nextDir;
        }
        keriseStr += "s"; cout << "keriseStr: " << keriseStr << endl;
    }
    return 0;
}
