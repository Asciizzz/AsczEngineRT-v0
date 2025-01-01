#ifndef CSLOGHANDLER_CUH
#define CSLOGHANDLER_CUH

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <string>

struct CsLog {
    std::string log;
    sf::Color color;
    short style;
};

class CsLogHandler {
public:
    CsLogHandler(std::string font="SpaceMono");

    // Styles
    int marginL = 10;
    int marginT = 10;
    int fontSize = 20;
    std::vector<sf::Font> fonts = std::vector<sf::Font>(4);

    // Log handling
    bool displayLog = true;
    std::vector<CsLog> cslogs;

    // Add/Clear log
    void addLog(std::string log, sf::Color color=sf::Color::White, short style=0);
    void addLog(std::string log, short style);
    void clear();

    // Draw log
    void drawLog(sf::RenderWindow& window);
};

#endif