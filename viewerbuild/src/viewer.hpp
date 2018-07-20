

#ifndef VIEWER_H
#define VIEWER_H

#include <mutex>
#include "drawer.hpp"

class Drawer;
class Viewer{
public:
    Viewer(std::map<std::string,std::string> commandlineArgs,Drawer& drawer);
    void Run();

private:
    Drawer& m_drawer;
};


#endif // VIEWER_H