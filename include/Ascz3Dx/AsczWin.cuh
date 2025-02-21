#ifndef ASCZWIN_CUH
#define ASCZWIN_CUH

#define WIN32_LEAN_AND_MEAN  // Reduce Windows header bloat

#define UNICODE
#define _UNICODE

#include <Vector.cuh>
#include <windows.h>

class AsczWin {
public:
    // Window properties
    int width, height;
    std::wstring title;
    HWND hwnd;
    HDC hdc;
    BITMAPINFO bmi;

    // Input state handling
    POINT mousePos = { 0, 0 };
    bool leftMouseDown = false;
    bool rightMouseDown = false;
    bool keys[256] = { false };

    // Framebuffers
    unsigned int* h_framebuffer;
    unsigned int* d_framebuffer;
    int threadCount = 256;
    int blockCount;

    // Debugs
    std::vector<std::string> debug_lines;
    int debug_line_count = 0;
    void appendDebug(std::string str, int line_count = 1) {
        debug_lines.push_back(str);
        debug_line_count += line_count;
    }
    void displayDebug() {
        if (debug_line_count == 0) return;

        std::cout << "\033[" << debug_line_count << "A";  // Move cursor up
        for (int i = 0; i < debug_line_count; i++) {
            std::cout << debug_lines[i] << "\n";
        }
        std::cout << std::flush;

        // Clear debug lines
        debug_lines.clear();
        debug_line_count = 0;
    }

    // Constructor
    AsczWin(int w, int h, std::wstring t);
    void InitConsole();
    void InitWindow();
    void InitGDI();

    // Draw Framebuffer
    void Draw();
    // Destroy Window
    void Terminate();

    // 📦 Static Window Procedure
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
};

#endif