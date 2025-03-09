#ifndef ASCZWIN_CUH
#define ASCZWIN_CUH

#define WIN32_LEAN_AND_MEAN  // Reduce Windows header bloat

#define UNICODE
#define _UNICODE

#include <Vector.cuh>
#include <windows.h>

struct AsczDebug {
    std::wstring text;
    Int3 color;
    int offx = 0;
};

class AsczWin {
public:
    // Window properties
    int width, height;
    std::wstring title;
    HWND hwnd;
    HDC hdc;
    BITMAPINFO bmi;

    // Input state handling

    // Mouse state
    POINT mousePos = { 0, 0 };
    bool leftMouseDown = false;
    bool rightMouseDown = false;
    // Keyboard state
    bool keys[256] = { false };
    // Scroll state
    int scroll = 0;

    // Constructor
    AsczWin(int w, int h, std::wstring t);
    void InitWindow();
    void InitGDI();

    // Debug
    std::vector<AsczDebug> debugs;

    void DrawTxt(HDC hdc, int x, int y, const AsczDebug &db);
    void appendDebug(std::wstring text, Int3 color=255, int offx=0);

    // Draw
    void Draw(unsigned int *draw, bool debug=true);

    // Destroy Window
    void Terminate();

    // 📦 Static Window Procedure
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
};

#endif