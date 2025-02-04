#include <CsLogHandler.cuh>

// The log handle
CsLogHandler::CsLogHandler(std::string font) {
    // For some reason relative path doesn't work
    // Lets just use the absolute path
    std::string dir = "assets/Fonts/" + font;
    fonts[0].loadFromFile(dir + "-Regular.ttf");
    fonts[1].loadFromFile(dir + "-Bold.ttf");
    fonts[2].loadFromFile(dir + "-Italic.ttf");
    fonts[3].loadFromFile(dir + "-BoldItalic.ttf");
}

// Add log
void CsLogHandler::addLog(std::string log, sf::Color color, short style) {
    CsLog cslog = {log, color, style};
    cslogs.push_back(cslog);
}
void CsLogHandler::addLog(std::string log, short style) {
    addLog(log, sf::Color::White, style);
}
// Clear log
void CsLogHandler::clear() {
    cslogs.clear();
}

// Draw log
void CsLogHandler::drawLog(sf::RenderWindow& window) {
    if (!displayLog) {
        cslogs.clear();
        return;
    };

    int count = 0;
    for (int i = 0; i < cslogs.size(); ++i) {
        // Remove the /n at the end of the log
        // As it is unnecessary
        if (cslogs[i].log.back() == '\n')
            cslogs[i].log.pop_back();

        // Count the number of /n in the previous logs
        if (i != 0) count += std::count(
            cslogs[i - 1].log.begin(), cslogs[i - 1].log.end(), '\n'
        );

        sf::Text text;
        text.setString(cslogs[i].log);
        text.setFont(fonts[cslogs[i].style]);
        text.setCharacterSize(fontSize);

        // A backdrop to highlight the text better
        text.setFillColor(sf::Color(255 - cslogs[i].color.r, 255 - cslogs[i].color.g, 255 - cslogs[i].color.b));
        text.setPosition(marginL + 1, marginT + fontSize*1.5 * (i + count) + 1);
        window.draw(text);
        text.setPosition(marginL + 1, marginT + fontSize*1.5 * (i + count) - 1);
        window.draw(text);
        text.setPosition(marginL - 1, marginT + fontSize*1.5 * (i + count) + 1);
        window.draw(text);
        text.setPosition(marginL - 1, marginT + fontSize*1.5 * (i + count) - 1);
        window.draw(text);

        // The main text
        text.setFillColor(cslogs[i].color);
        text.setPosition(marginL, marginT + fontSize*1.5 * (i + count));
        window.draw(text);
    }

    cslogs.clear();
}