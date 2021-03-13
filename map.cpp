#include "map.h"
using namespace tinyxml2;

Map::Map()
{
    height = 0;
    width = 0;
}
Map::~Map()
{	
    Grid.clear();
}

bool Map::getMap(const char* FileName)
{
    XMLDocument doc;
    if(doc.LoadFile(FileName) != XMLError::XML_SUCCESS)
    {
        std::cout << "Error openning input XML file."<<std::endl;
        return false;
    }

    XMLElement *root = nullptr;
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "No '"<<CNS_TAG_ROOT<<"' element found in XML map-file."<<std::endl;
        return false;
    }

    XMLElement *map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "No '"<<CNS_TAG_MAP<<"' element found in XML file."<<std::endl;
        return false;
    }

    XMLElement *grid = map->FirstChildElement(CNS_TAG_GRID);
    if (!grid)
    {
        std::cout << "No '"<<CNS_TAG_GRID<<"' element found in XML file."<<std::endl;
        return false;
    }
    height = grid->IntAttribute(CNS_TAG_ATTR_HEIGHT);
    if(height <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_HEIGHT<<" attribute. It should be >0.\n";
        return false;
    }
    width = grid->IntAttribute(CNS_TAG_ATTR_WIDTH);
    if(width <= 0)
    {
        std::cout<<"Wrong value of "<<CNS_TAG_ATTR_WIDTH<<" attribute. It should be >0.\n";
        return false;
    }
    XMLElement *row = grid->FirstChildElement(CNS_TAG_ROW);
    Grid.resize(height);
    for(int i = 0; i < height; i++)
        Grid[i].resize(width, 0);

    std::string value;
    const char* rowtext;
    std::stringstream stream;
    for(int i = 0; i < height; i++)
    {
        if (!row)
        {
            std::cout << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given." << std::endl;
            return false;
        }

        rowtext = row->GetText();
        unsigned int k = 0;
        value = "";
        int j = 0;

        for(k = 0; k < strlen(rowtext); k++)
        {
            if (rowtext[k] == ' ')
            {
                stream << value;
                stream >> Grid[i][j];
                stream.clear();
                stream.str("");
                value = "";
                j++;
            }
            else
            {
                value += rowtext[k];
            }
        }
        stream << value;
        stream >> Grid[i][j];
        stream.clear();
        stream.str("");

        if (j < width-1)
        {
            std::cout << "Not enough cells in '" << CNS_TAG_ROW << "' " << i << " given." << std::endl;
            return false;
        }
        row = row->NextSiblingElement(CNS_TAG_ROW);
    }
    this->generateMoves();
    return true;
}


bool Map::CellIsTraversable(int i, int j) const
{
    return (Grid[i][j] == 0);
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != 0);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

int Map::getValue(int i, int j) const
{
    if(i < 0 || i >= height)
        return -1;
    if(j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}
void Map::generateMoves()
{
    std::vector<std::pair<int, int>> quad = {{1, 0}, {0, 1}};
    for (int i = 2; i < 100; ++i) {
        std::vector<std::pair<int, int>> newQuad;
        for (auto it1 = quad.begin(); it1 != quad.end(); ++it1) {
            newQuad.push_back(*it1);
            if (std::next(it1) != quad.end()) {
                int sum_i = it1->first + std::next(it1)->first;
                int sum_j = it1->second + std::next(it1)->second;
                if(sum_i < height && sum_j < width)
                    newQuad.push_back(std::make_pair(sum_i, sum_j));
            }
        }
        quad = newQuad;
    }

    std::set<std::pair<int, int>> neigh;
    for (auto pair : quad) {
        neigh.insert(pair);
        neigh.insert(std::make_pair(-pair.first, pair.second));
        neigh.insert(std::make_pair(pair.first, -pair.second));
        neigh.insert(std::make_pair(-pair.first, -pair.second));
    }
    for(auto n:neigh)
        moves.push_back(Node(n.first, n.second, sqrt(n.first*n.first + n.second*n.second)));
}

std::vector<Node> Map::getValidMoves(int i, int j, int k, double size) const
{
   LineOfSight los;
   los.setSize(size);
   std::vector<Node> moves;
   if(k == 2)
       moves = {Node(0,1,1.0),   Node(1,0,1.0),         Node(-1,0,1.0), Node(0,-1,1.0)};
   else if(k == 3)
       moves = {Node(0,1,1.0),   Node(1,1,sqrt(2.0)),   Node(1,0,1.0),  Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),  Node(-1,-1,sqrt(2.0)), Node(-1,0,1.0), Node(-1,1,sqrt(2.0))};
   else if(k == 4)
       moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0))};
   else if(k == 5)
       moves = {Node(0,1,1.0),          Node(1,1,sqrt(2.0)),    Node(1,0,1.0),          Node(1,-1,sqrt(2.0)),
                Node(0,-1,1.0),         Node(-1,-1,sqrt(2.0)),  Node(-1,0,1.0),         Node(-1,1,sqrt(2.0)),
                Node(1,2,sqrt(5.0)),    Node(2,1,sqrt(5.0)),    Node(2,-1,sqrt(5.0)),   Node(1,-2,sqrt(5.0)),
                Node(-1,-2,sqrt(5.0)),  Node(-2,-1,sqrt(5.0)),  Node(-2,1,sqrt(5.0)),   Node(-1,2,sqrt(5.0)),
                Node(1,3,sqrt(10.0)),   Node(2,3,sqrt(13.0)),   Node(3,2,sqrt(13.0)),   Node(3,1,sqrt(10.0)),
                Node(3,-1,sqrt(10.0)),  Node(3,-2,sqrt(13.0)),  Node(2,-3,sqrt(13.0)),  Node(1,-3,sqrt(10.0)),
                Node(-1,-3,sqrt(10.0)), Node(-2,-3,sqrt(13.0)), Node(-3,-2,sqrt(13.0)), Node(-3,-1,sqrt(10.0)),
                Node(-3,1,sqrt(10.0)),  Node(-3,2,sqrt(13.0)),  Node(-2,3,sqrt(13.0)),  Node(-1,3,sqrt(10.0))};

   std::vector<bool> valid(moves.size(), true);
   std::vector<Node> v_moves = {};
   if(moves.empty())
   {
       int h = height, w = width;
       for(int i1=-h; i1<h; i1++)
           for(int j1=-w; j1<w; j1++)
           {
               if(CellOnGrid(i+i1,j+j1))
                   if(!CellIsObstacle(i+i1,j+j1))
                       if(!(i1==0 && j1==0))
                           if(los.checkLine(i, j, i+i1, j+j1, *this))
                                v_moves.push_back(Node(i1,j1, sqrt(i1*i1+j1*j1)));
           }
       /*valid.resize(this->moves.size(), true);
       for(int k = 0; k < this->moves.size(); k++)
       {
           if(!CellOnGrid(i + this->moves[k].i, j + this->moves[k].j) || CellIsObstacle(i + this->moves[k].i, j + this->moves[k].j)
                   || !los.checkLine(i, j, i + this->moves[k].i, j + this->moves[k].j, *this))
               valid[k] = false;
       }
       for(int k = 0; k < valid.size(); k++)
           if(valid[k])
               v_moves.push_back(this->moves[k]);*/
   }
   else
   {
       for(int k = 0; k < moves.size(); k++)
           if(!CellOnGrid(i + moves[k].i, j + moves[k].j) || CellIsObstacle(i + moves[k].i, j + moves[k].j)
                   || !los.checkLine(i, j, i + moves[k].i, j + moves[k].j, *this))
               valid[k] = false;
       for(int k = 0; k < valid.size(); k++)
           if(valid[k])
               v_moves.push_back(moves[k]);
   }
   return v_moves;
}
