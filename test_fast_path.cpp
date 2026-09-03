#include <iostream>
#include <string>
#include <vector>
#include "lib/maze/include/MazeLib/Maze.h"
#include "lib/maze/include/MazeLib/SearchAlgorithm.h"

using namespace std;

void replace(string& src, const string& search, const string& replace_str) {
    size_t pos = 0;
    while ((pos = src.find(search, pos)) != string::npos) {
        src.replace(pos, search.length(), replace_str);
        pos += replace_str.length();
    }
}

string convert_search_to_fast(string src) {
    replace(src, "RF", "R");
    replace(src, "LF", "L");
    replace(src, "BF", "B");
    replace(src, "F", "S");

    // 1. Mở rộng (Expand)
    replace(src, "S", "ss"); 
    replace(src, "L", "ll"); 
    replace(src, "R", "rr"); 
    
    replace(src, "rllllr", "rlplr"); // FV90_L (p)
    replace(src, "lrrrrl", "lrPrl"); // FV90_R (P)
    replace(src, "sllr", "zlr");     // F45_L (z)
    replace(src, "srrl", "crl");     // F45_R (c)
    replace(src, "rlls", "rlZ");     // F45_LP (Z)
    replace(src, "lrrs", "lrC");     // F45_RP (C)
    replace(src, "sllllr", "alr");   // F135_L (a)
    replace(src, "srrrrl", "drl");   // F135_R (d)
    replace(src, "rlllls", "rlA");   // F135_LP (A)
    replace(src, "lrrrrs", "lrD");   // F135_RP (D)
    replace(src, "slllls", "u");     // F180_L (u)
    replace(src, "srrrrs", "U");     // F180_R (U)
    replace(src, "rllr", "rlwlr");   // ST_DIAG (w)
    replace(src, "lrrl", "lrwrl");   // ST_DIAG (w)
    replace(src, "slls", "q");       // F90_L (q)
    replace(src, "srrs", "Q");       // F90_R (Q)
    
    replace(src, "rl", "");      
    replace(src, "lr", "");      
    
    replace(src, "ss", "S");     // ST_FULL (S)
    replace(src, "ll", "L");     // FS90_L (L)
    replace(src, "rr", "R");     // FS90_R (R)
    replace(src, "s", "h");      // ST_HALF (h)
    
    return src;
}

int main() {
    string mapHex = "MAP:0010D556C44444444446D553800000000002C4440000000000028000000000000002800000000000000280000000000000028000000000000002800000000000000280000000000000028000000000000002800000000000000280000000000000028000000000000002800000000000000280000000000000029111111111111113";
    mapHex = mapHex.substr(4);
    
    int startX = (mapHex[0] >= 'A') ? (mapHex[0] - 'A' + 10) : (mapHex[0] - '0');
    int startY = (mapHex[1] >= 'A') ? (mapHex[1] - 'A' + 10) : (mapHex[1] - '0');
    int goalX  = (mapHex[2] >= 'A') ? (mapHex[2] - 'A' + 10) : (mapHex[2] - '0');
    int goalY  = (mapHex[3] >= 'A') ? (mapHex[3] - 'A' + 10) : (mapHex[3] - '0');

    MazeLib::Positions goals;
    goals.push_back(MazeLib::Position(goalX, goalY));
    MazeLib::Maze maze(goals, MazeLib::Position(startX, startY));
    maze.reset();
    
    int idx = 4;
    for (int x = 0; x < 16; x++) {
        for (int y = 0; y < 16; y++) {
            char c = mapHex[idx++];
            uint8_t val = 0;
            if (c >= '0' && c <= '9') val = c - '0';
            else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
            
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
    
    MazeLib::SearchAlgorithm searcher(maze);
    MazeLib::Directions shortestPath;
    bool success = searcher.calcShortestDirections(shortestPath, true);

    if (success) {
        string keriseStr = "";
        if (!shortestPath.empty()) {
            MazeLib::Direction prevDir = shortestPath[0];
            for (size_t i = 1; i < shortestPath.size(); ++i) {
                MazeLib::Direction nextDir = shortestPath[i];
                int diff = (nextDir - prevDir + 8) % 8;
                if (diff == 0) keriseStr += "S";
                else if (diff == 6) keriseStr += "R";
                else if (diff == 2) keriseStr += "L";
                else if (diff == 4) keriseStr += "B";
                prevDir = nextDir;
            }
        }
        keriseStr = "s" + keriseStr + "s";
        cout << "keriseStr: " << keriseStr << endl;
        cout << "fast_path: " << convert_search_to_fast(keriseStr) << endl;
    } else {
        cout << "Search failed!" << endl;
    }
    return 0;
}
